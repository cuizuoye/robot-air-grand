"""Offline candidate codec. No serial I/O, firmware deployment or flight permission.

One fixed-length record can fit a MAVLink V2_EXTENSION payload. Transport ID is not
assigned here: raw records must never be treated as authenticated telemetry.
Timestamps are original uORB microseconds, never serialization time.
"""
import math
import struct

FLAGS = ('cs_tilt_align','cs_baro_hgt','cs_ev_pos','cs_ev_hgt','cs_ev_yaw',
         'cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_in_air','cs_ev_vel',
         'fs_bad_pos_n','fs_bad_pos_e','fs_bad_pos_d','fs_bad_hdg',
         'reject_hor_pos','reject_ver_pos','reject_yaw')
HEAD = struct.Struct('<4sQQIBBQBB')
AID = struct.Struct('<QQQBBdddd')
SIZE = HEAD.size + 3*AID.size


class AidNotReady(ValueError):
    """Well-formed PX4 inactive/rejected aid, never successful evidence.

    PX4 can publish +Inf test ratios before an aiding source is enabled.
    Preserve the record for structural validation, but never promote it to a
    fresh fusion sample. Other nonfinite values remain protocol failures.
    """
    def __init__(self, row, fields):
        super().__init__('Inactive rejected aid not ready: '+', '.join(fields))
        self.row, self.fields = row, tuple(fields)


def _integer(v, bits):
    if type(v) is not int or not 0 <= v < 2**bits:
        raise ValueError('Invalid integer')
    return v


def _mask(values, names):
    if any(type(values[n]) is not bool for n in names):
        raise ValueError('Boolean fields required')
    return sum(int(values[n]) << i for i,n in enumerate(names))


def encode(row):
    f,l,s=row['flags'],row['local'],row['selector']
    primary=_integer(s['primary_instance'],8)
    count=_integer(s['instances_available'],8)
    if not 1 <= count <= 9 or primary >= count:
        raise ValueError('Invalid selector')
    out=HEAD.pack(b'EVF1',_integer(s['timestamp'],64),_integer(f['timestamp'],64),
        _mask(f,FLAGS),primary,count,_integer(l['timestamp'],64),
        _mask(l,('xy_valid','z_valid')),
        _mask(s,('primary_healthy','gyro_fault_detected','accel_fault_detected')))
    for key,dim in (('aid',2),('height_aid',1),('yaw_aid',1)):
        a=row[key]
        ratios=a['test_ratio'] if dim==2 else [a['test_ratio'],0.]
        variances=a['observation_variance'] if dim==2 else [a['observation_variance'],0.]
        if len(ratios)!=2 or len(variances)!=2 or not all(math.isfinite(v) for v in ratios+variances):
            raise ValueError('Invalid numeric evidence')
        out+=AID.pack(*(_integer(a[n],64) for n in ('timestamp','timestamp_sample','time_last_fuse')),
            _integer(a['estimator_instance'],8),
            _mask(a,('fusion_enabled','fused','innovation_rejected')),*ratios,*variances)
    return out


def decode(raw):
    if len(raw)!=SIZE: raise ValueError('Exact record length required')
    magic,st,ft,mask,primary,count,lt,local,health=HEAD.unpack_from(raw)
    if magic!=b'EVF1' or mask>>len(FLAGS) or local>>2 or health>>3 or not 1<=count<=9 or primary>=count:
        raise ValueError('Unknown schema or reserved bits')
    row={'flags':dict(timestamp=ft,**{n:bool(mask&(1<<i)) for i,n in enumerate(FLAGS)}),
         'local':dict(timestamp=lt,xy_valid=bool(local&1),z_valid=bool(local&2)),
         'selector':dict(timestamp=st,primary_instance=primary,instances_available=count,
             primary_healthy=bool(health&1),gyro_fault_detected=bool(health&2),accel_fault_detected=bool(health&4))}
    unready = []
    for i,(key,dim,flag) in enumerate((('aid',2,'cs_ev_pos'),('height_aid',1,'cs_ev_hgt'),('yaw_aid',1,'cs_ev_yaw'))):
        t,ts,tf,instance,bits,r1,r2,v1,v2=AID.unpack_from(raw,HEAD.size+i*AID.size)
        if bits>>3 or instance>=9:
            raise ValueError('Invalid aid evidence')
        if dim==1 and (r2!=0 or v2!=0): raise ValueError('Nonzero scalar padding')
        if not all(math.isfinite(v) for v in (r1,r2,v1,v2)):
            # A real source record may carry +Inf while explicitly inactive,
            # not fused and rejected. Do not normalize, accept as healthy, or
            # treat NaN/negative infinity/nonfinite variance the same way.
            if (bits == 4 and not row['flags'][flag]
                    and all(math.isfinite(v) for v in (v1,v2))
                    and all(math.isfinite(v) or v == math.inf for v in (r1,r2))):
                unready.append(key)
            else:
                raise ValueError('Invalid aid evidence: '+key)
        row[key]=dict(timestamp=t,timestamp_sample=ts,time_last_fuse=tf,estimator_instance=instance,
            fusion_enabled=bool(bits&1),fused=bool(bits&2),innovation_rejected=bool(bits&4),
            test_ratio=[r1,r2] if dim==2 else r1,observation_variance=[v1,v2] if dim==2 else v1)
    if unready:
        raise AidNotReady(row, unready)
    return row
