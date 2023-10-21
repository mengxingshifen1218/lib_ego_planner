#include "dyn_a_star.h"

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            delete GridNodeMap_[i][j];
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector2i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr *[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr [POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNode;
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    double h = 0.0;
    int diag = min(dx, dy);
    dx -= diag;
    dy -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + 1.0 * abs(dy);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + 1.0 * abs(dx);
    }

    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));

    return dx + dy;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector2d start_pt, Vector2d end_pt, Vector2i &start_idx, Vector2i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    int occ;
    if (checkOccupancy(Index2Coord(start_idx)))
    {
        // printf("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            // cout << "start_pt=" << start_pt.transpose() << endl;
            if (!Coord2Index(start_pt, start_idx))
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));
            if (occ == -1)
            {
                printf("[Astar] Start point outside the map region.");
                return false;
            }
        } while (occ);
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        // printf("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            // cout << "end_pt=" << end_pt.transpose() << endl;
            if (!Coord2Index(end_pt, end_idx))
            {
                return false;
            }

            occ = checkOccupancy(Index2Coord(start_idx));
            if (occ == -1)
            {
                printf("[Astar] End point outside the map region.");
                return false;
            }
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector2d start_pt, Vector2d end_pt)
{
    auto time_1 = ego_planner::Now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Vector2i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        printf("Unable to handle the initial or end point, force return!");
        return ASTAR_RET::INIT_ERR;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; // put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); // put start in open set

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1))
        {
            // autotime_2 = Now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).count()*1000);
            // if((time_2 - time_1).count() > 0.1)
            //     printf("Time consume in A star path finding is %f", (time_2 - time_1).count() );
            gridPath_ = retrievePath(current);
            return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; // move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector2i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1)
                {
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)];
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    continue; // in closed set.
                }

                neighborPtr->rounds = rounds_;

                if (checkOccupancy(Index2Coord(neighborPtr->index)))
                {
                    continue;
                }

                double static_cost = sqrt(dx * dx + dy * dy);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    // discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push(neighborPtr); // put neighbor in open set and record it.
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { // in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                }
            }
        auto time_2 = ego_planner::Now();
        if ((time_2 - time_1).count() / 1e9 > 0.2)
        {
            printf("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    auto time_2 = ego_planner::Now();

    if ((time_2 - time_1).count() / 1e9 > 0.1)
        printf("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).count() / 1e9, num_iter);

    return ASTAR_RET::SEARCH_ERR;
}

vector<Vector2d> AStar::getPath()
{
    vector<Vector2d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
