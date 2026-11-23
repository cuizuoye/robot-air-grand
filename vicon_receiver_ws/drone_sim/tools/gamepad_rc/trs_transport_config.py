"""Explicit host UART selection only; never probes or configures a radio/FC."""

BAUDS = (57600, 115200)


def require_baud(value):
    if type(value) is not int or value not in BAUDS:
        raise ValueError('Explicit TRS baud 57600 or 115200 required; no automatic detection or fallback')
    return value


def add_baud_argument(parser):
    parser.add_argument('--baud', type=int, choices=BAUDS, required=True,
                        help='Known host TRS baud; does not change FC/radio settings or try fallback speeds')
