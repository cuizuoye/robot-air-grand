// Separate read-only SDK connection. No FC access, server configuration,
// axis mapping writes, shared receiver restart or motion commands.
#include <vicon-datastream-sdk/DataStreamClient.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

using namespace ViconDataStreamSDK::CPP;

template <size_t N> void array(std::ostream &out, const double (&values)[N]) {
    out << '[';
    for (size_t i=0;i<N;++i) { if(i)out << ','; out << values[i]; }
    out << ']';
}

int main(int argc, char **argv) {
    if (argc < 2 || std::filesystem::exists(argv[1])) {
        std::cerr << "Usage: snapshot NEW.json [SUBJECT ...]\n"; return 2;
    }
    std::vector<std::string> subjects;
    for (int i=2;i<argc;++i) subjects.emplace_back(argv[i]);
    if (subjects.empty()) subjects.emplace_back("MircoAir");
    const int requested=subjects.size()>1 ? 200:20;
    Client client;
    if (client.Connect("10.10.10.2").Result != Result::Success) {
        std::cerr << "Read-only Vicon connection failed\n"; return 2;
    }
    client.EnableSegmentData();
    client.EnableMarkerData(); // Geometry from the same frame as pose.
    client.SetStreamMode(StreamMode::ClientPull); // This client only.
    const auto axes=client.GetAxisMapping();
    std::ofstream out(argv[1]);
    if (!out) { std::cerr << "Cannot create output\n"; return 2; }
    out << std::setprecision(17);
    out << "{\"scope\":\"Separate read-only SDK snapshot; no FC or shared receiver changes\","
        << "\"axis_mapping_enum\":[" << int(axes.XAxis) << ',' << int(axes.YAxis) << ',' << int(axes.ZAxis)
        << "],\"rows\":[";
    int samples=0, invalid_frames=0;
    bool first_row=true;
    unsigned int last_frame=0;
    const auto started=std::chrono::steady_clock::now();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (samples < requested && std::chrono::steady_clock::now()<deadline) {
        if (client.GetFrame().Result != Result::Success) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue;
        }
        const auto frame=client.GetFrameNumber().FrameNumber;
        if (samples && frame==last_frame) continue;
        last_frame=frame;
        std::vector<std::string> segments;
        bool valid=true;
        for (const auto &subject:subjects) {
            const auto root=client.GetSubjectRootSegmentName(subject);
            if (root.Result!=Result::Success) { valid=false; break; }
            segments.emplace_back(std::string(root.SegmentName));
            const auto q=client.GetSegmentGlobalRotationQuaternion(subject,segments.back());
            const auto p=client.GetSegmentGlobalTranslation(subject,segments.back());
            const auto r=client.GetSegmentGlobalRotationMatrix(subject,segments.back());
            const auto e=client.GetSegmentGlobalRotationEulerXYZ(subject,segments.back());
            if (q.Result!=Result::Success || p.Result!=Result::Success || r.Result!=Result::Success
                || e.Result!=Result::Success || q.Occluded || p.Occluded || r.Occluded || e.Occluded) {
                valid=false; break;
            }
        }
        if (!valid) { ++invalid_frames; continue; }
        // All subjects below come from this single GetFrame(), never time-paired
        // from separate clients or restamped from cached ROS messages.
        const auto received=std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        for (size_t s=0;s<subjects.size();++s) {
        const auto &subject=subjects[s]; const auto &segment=segments[s];
        const auto q=client.GetSegmentGlobalRotationQuaternion(subject, segment);
        const auto r=client.GetSegmentGlobalRotationMatrix(subject, segment);
        const auto e=client.GetSegmentGlobalRotationEulerXYZ(subject, segment);
        const auto p=client.GetSegmentGlobalTranslation(subject, segment);
        if(!first_row) out << ',';
        first_row=false;
        out << "{\"frame\":" << frame << ",\"received_monotonic_ns\":" << received
            << ",\"subject\":" << std::quoted(subject) << ",\"segment\":" << std::quoted(segment)
            << ",\"quaternion_xyzw\":";
        array(out,q.Rotation); out << ",\"rotation_matrix\":"; array(out,r.Rotation);
        out << ",\"euler_xyz_rad\":"; array(out,e.Rotation);
        out << ",\"translation_mm\":"; array(out,p.Translation);
        const auto mc=client.GetMarkerCount(subject);
        out << ",\"marker_count_result\":" << int(mc.Result) << ",\"markers\":[";
        if (mc.Result==Result::Success) {
            for (unsigned int i=0;i<mc.MarkerCount;++i) {
                if(i) out << ',';
                const auto name=client.GetMarkerName(subject,i);
                const auto marker=client.GetMarkerGlobalTranslation(subject,name.MarkerName);
                out << "{\"name\":" << std::quoted(std::string(name.MarkerName))
                    << ",\"name_result\":" << int(name.Result)
                    << ",\"position_result\":" << int(marker.Result)
                    << ",\"occluded\":" << (marker.Occluded ? "true":"false")
                    << ",\"translation_mm\":";
                array(out,marker.Translation); out << '}';
            }
        }
        out << "]}";
        }
        ++samples;
    }
    out << "],\"samples\":" << samples << ",\"requested_frames\":" << requested
        << ",\"invalid_frames\":" << invalid_frames << ",\"duration_s\":"
        << std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count() << "}\n";
    out.close();
    client.Disconnect();
    std::cout << "Read-only SDK samples=" << samples << "\n";
    return samples==requested ? 0:2;
}
