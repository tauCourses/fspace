#include <fstream>
#include <sstream>
#include <set>
#include <CGAL/Gmpq.h>
#include <CGAL/Cartesian.h>
#include <utility>

typedef CGAL::Cartesian<CGAL::Gmpq> Kernel;
typedef Kernel::Point_2 point;
typedef Kernel::Line_2 line;
typedef Kernel::Intersect_2 intersect;

using namespace std;

point read_point(std::ifstream &is) {
    std::vector<Kernel::FT> point_to_be;
    for (int i = 0; i < 2; ++i) {
        Kernel::FT c;
        is >> c;

        point_to_be.push_back(c);
    }
    return {point_to_be[0],point_to_be[1]};
}

class segment{
public:
    segment(point start, point end) : start(std::move(start)), end(std::move(end))
    {}
    point start, end;
};

ostream& operator<< (ostream& os, const segment& obj) {
    os << "start " << obj.start << " end " << obj.end;
    return os;
}

vector<segment> getRoomSegments(vector<point>& room_points)
{
    vector<segment> segments;
    for(int i=1; i<room_points.size(); i++)
        segments.emplace_back(room_points[i-1], room_points[i]);
    segments.emplace_back(room_points[room_points.size()-1], room_points[0]);
    return segments;
}

bool betterPoint(segment& seg, point &base, point &check)
{
    return (seg.end[0]-seg.start[0])*(check[1] - base[1]) - (check[0]-base[0])*(seg.end[1]-seg.start[1]) > 0;
}

int getBestIndexForSegment(vector<point>& robot_points, segment &seg)
{
    int bestOption = 0;
    for(int i=1; i<robot_points.size();i++)
        if(betterPoint(seg, robot_points[bestOption],robot_points[i]))
            bestOption = i;
    //cout << seg << endl;
    //cout << "best point - " << robot_points[bestOption] << endl;
    return bestOption;
}

int getNextIndexForSegment(vector<point>& robot_points, int currentRobotPoint, segment &seg)
{
    int bestOption = currentRobotPoint;
    while(true)
    {
        int next = bestOption+1;
        if(next == robot_points.size())
            next = 0;
        if(betterPoint(seg, robot_points[bestOption], robot_points[next]))
            bestOption = next;
        else
            break;

    }
    //cout << seg << endl;
    //cout << "best point - " << robot_points[bestOption] << endl;
    return bestOption;
}

segment getSpaceSegment(segment& seg, point& limit, point &ref)
{
    return segment({seg.start[0] + ref[0] - limit[0], seg.start[1] + ref[1] - limit[1]},
                   {seg.end[0] + ref[0] - limit[0], seg.end[1] + ref[1] - limit[1]});
}

point findPointBetweenTwoSegment(segment &firstSeg, segment &secondSeg)
{
    line l1(firstSeg.start, firstSeg.end);
    line l2(secondSeg.start, secondSeg.end);
    CGAL::cpp11::result_of<intersect(line, line)>::type
            result =  intersection(l1, l2);

    point* p = boost::get<point>(&*result);
    if(!p)
        throw "not a point intersection!";
    return {(*p)[0],(*p)[1]};
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Wrong number of arguments" << std::endl;
        return 1;
    }
    const auto *room_filename = argv[1];

    std::ifstream room_is;
    room_is.open(room_filename);
    if (!room_is.is_open()) {
        std::cerr << "Failed to open " << room_filename << "!" << std::endl;
        return -1;
    }
    size_t room_n;
    room_is >> room_n;
    std::vector<point> room_points;

    for (size_t i = 0; i < room_n; ++i) {
        point p  = read_point(room_is);
        room_points.push_back(p);
    }

    const auto *robot_filename = argv[2];

    std::ifstream robot_is;
    robot_is.open(robot_filename);
    if (!robot_is.is_open()) {
        std::cerr << "Failed to open " << robot_filename << "!" << std::endl;
        return -1;
    }
    size_t robot_n;
    robot_is >> robot_n;
    vector<point> robot_points;

    for (size_t i = 0; i < robot_n; ++i) {
        point p  = read_point(robot_is);
        robot_points.push_back(p);
    }

    /*cout << "room: " << endl;
    for(auto& p:room_points)
        cout << p << " >> ";
    cout << endl;

    cout << "robot: " << endl;
    for(auto& p:robot_points)
        cout << p << " >> ";
    cout << endl;*/

    vector<segment> room_segments = getRoomSegments(room_points);

    int currentRobotPoint = getBestIndexForSegment(robot_points, room_segments[0]);
    vector<int> limitIndexes;
    limitIndexes.emplace_back(currentRobotPoint);
    for(int i=1; i<room_segments.size();i++)
    {
        int temp = getNextIndexForSegment(robot_points, currentRobotPoint, room_segments[i]);
        limitIndexes.emplace_back(temp);
        currentRobotPoint = temp;
    }

    point& reference_point = robot_points[0];
    //cout << "ref point " << reference_point << endl;
    vector<segment> freeSpaceSegment;
    //cout << "free segments:\n";
    for(int i=0; i<room_segments.size(); i++) {
        freeSpaceSegment.emplace_back(
                getSpaceSegment(room_segments[i], robot_points[limitIndexes[i]], reference_point));
        //cout << freeSpaceSegment[i] << endl;
    }

    vector<point> freeSpace;
    for(int i=0; i<room_segments.size()-1; i++)
        freeSpace.emplace_back(findPointBetweenTwoSegment(freeSpaceSegment[i], freeSpaceSegment[i+1]));
    freeSpace.emplace_back(findPointBetweenTwoSegment(freeSpaceSegment[room_segments.size()-1],
                                                      freeSpaceSegment[0]));

    //cout << "free points:\n";
    cout << freeSpace.size() << " ";
    for(auto& point: freeSpace)
        cout << point << " ";

    return 0;
}