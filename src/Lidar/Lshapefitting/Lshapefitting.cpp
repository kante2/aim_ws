#include <Lidar/Lshapefitting/Lshapefitting.hpp>

struct LshapefittingProcess
{
    void Calculate_Process (LidarCluster& st_LidarCluster);

    // -----------------------------------------------------------------------------

    bool SortingPoints (const Point2D& a, const Point2D& b);

    double CrossProduct (const Point2D& p1, const Point2D& p2, const Point2D& p3);
    
    // calculate convex 반환값을 뭐라고 해야하노 -> convex_hull 이루는 정점 집합
    vector <Point2D> Calculate_ConvexHull (vector<Point2D>& vec_Points);

    // calculate closeness 반환값은 시그마 1/d 한 beta? 
    // double Calculate_Closeness (LidarCluster& st_LidarCluster);
    double Calculate_Closeness(
    const vector<Point2D>& vec_Points,
    const vector<Point2D>& vec_Convexhull_Points, // (vec) convexhull 1개에 들어있는 points
    double& BestTheta,
    double& BestAobb); // 파라미터 좀 의문

};

// ========================================================================
// 클러스터 1개 안에서 진행되는 중간 계산 과정
// ========================================================================

// -----------------------   x, y 오름차순 정렬 함수  --------------------
bool LshapefittingProcess::SortingPoints(const Point2D& a, const Point2D& b) 
{
    if (a.x == b.x) return a.y < b.y;
    return a.x < b.x;
}

// 외적 계산 함수
double LshapefittingProcess::CrossProduct (const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

// 오름차순 정렬된 Point2D 에 대해 외적 계산한 결과 이용해서 convex hull 정점구하는 함수
vector <Point2D> LshapefittingProcess::Calculate_ConvexHull (vector<Point2D>& vec_Points)
{
    int32 n = (int32)vec_Points.size();
    if (n < 3) return vec_Points;

    // 1. 오름차순 정렬
    sort(vec_Points.begin(), vec_Points.end(), SortingPoints);

    vector<Point2D> vec_Convexhull_Points;

    // 2. lower Hull 계산
    for (int32 i = 0; i < n; i++)
    {
        while (vec_Convexhull_Points.size() >= 2)
        {
            if (CrossProduct(vec_Convexhull_Points[vec_Convexhull_Points.size() - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
                vec_Convexhull_Points.pop_back();
            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }

    // 3. upper Hull 계산
    size_t lower_size = vec_Convexhull_Points.size();
    for (int32 i = n - 2; i >= 0; --i) {
        while (vec_Convexhull_Points.size() > lower_size) {
            if (CrossProduct(vec_Convexhull_Points[vec_Convexhull_Points.size() - 2], vec_Convexhull_Points.back(), vec_Points[i]) <= 0) 
                vec_Convexhull_Points.pop_back();
            else break;
        }
        vec_Convexhull_Points.push_back(vec_Points[i]);
    }
    vec_Convexhull_Points.pop_back();

    return vec_Convexhull_Points; // 외곽선 정점 집합 반환
}

// 
double LshapefittingProcess::Calculate_Closeness (
    const vector<Point2D>& vec_Points,
    const vector<Point2D>& vec_Convexhull_Points, // (vec) convexhull 1개에 들어있는 points
    double& BestTheta,
    double& BestAobb)
{
    // st_LidarCluster 에 있는 p = (x, y) 에 대해 
    // d = min (|x - rotate_rect_min_x|, |rotate_rect_max_x - x|, |y - miny|, |maxy - y|)
    // score += 1/d
    
    // 1. 외적으로 후보 theta 구하고
    // 2. theta 하나 당 rotate (축정렬) 
    // (for 정점 순회 1번)
    // 3. 모든 정점을 포함하는 직사각형 생성 (min,max 구해짐)
    // (for 정점 순회 1번 더)
    // 4. convex_hull 을 이루는 모든 정점들에 대해 정점 하나씩 순회하면서  min,max (직사각형 꼭짓점) 까지의 
    //    dist 를 구해서 가장 작은 dist를 저장. 
    // 5. 이걸 모든 점에 대해 반복. 1/d 을 더해서 각 theta 에 대한 score로 저장  
}

// ===============================================================
// struct에 있는 중간 계산 함수들 포함해서 클러스터 1개에서 진행되는 계산 총괄
// ===============================================================

void LshapefittingProcess::Calculate_Process (LidarCluster& st_LidarCluster)
{
    // 1. cluster 점 -> z 자르고 (x, y) 로만 저장 -> vector<Point2D> 생성
    // 2. Calculate_ConvexHull 실행
    // -> convexhull 이루는 정점 도출

    // 3. 정점 이용해서 hull area 계산 (함수x) = Ahull 에 저장. 

    // 4. Calculate_Closeness 실행
    // -> theta 에 대한 score = 베타(theta)

    // 5. 최적 직사각형 (꼭짓점을 closeness에서 계산 -> 반환) 넓이 Aobb 계산
    // 6. Ahull/Aobb+abs = 감마(theta)

    // 7. 클러스터 1개에 대한 score = 베타 * 감마


    // ===== LShapeFitting 함수에서 작성 =====

    // 8. max score -> best theta. 
    //    -> struct LidarCluster 에 저장. (직사각형의 꼭짓점 => costmap 에서의 위치)

}


// ===============================================================
// 여러 클러스터 돌면서 convex hull의 전체 후보 각도에 대해 직사각형 계산하고, 최적의 직사각형 피팅
// ===============================================================

// 이중 for문으로 클러스터 하나에서 쭉 계산(Calculate_Process) 반복
void LShapeFitting (Lidar& st_Lidar)
{
    
    // 입력 : vector<pcl::PointIndices> indices;
    pcl::PointCloud<pcl::PointXYZI> pcl_allclusterpoint;
    // 클러스터링 할 때 클러스터 단위로 포인트들을 분리해서 저장해두고, 여기에서는 그 포인트 (인덱스X)에 접근해서 쓰는 걸로
    for (auto& pcl_cluster : st_Lidar.vec_cluster) 
    {

    }

}
