#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include "globalVariables.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class LpaStar{

public:
    LpaStar();
    void Initilize(GridWorld &gWorld);
    int Search();
    ~LpaStar();

private:
    void adjust_heap(int nIndex);
    void swap(int idx1, int idx2);
    void get_neighbors(LpaStarCell* curr_node, LpaStarCell* end_node);
    void insert_to_opentable(int x, int y, LpaStarCell *curr_node, LpaStarCell *end_node, int w);
    
    
    LpaStarCell* open_table[100];    // open表
    LpaStarCell* close_table[100];   // close表
    int open_node_count;            // open表中节点数量
    int close_node_count;           // close表中结点数量
    LpaStarCell* path_stack[100];    // 保存路径的栈
    int top;
    vector<vector<LpaStarCell> > maze;
	int rows;
    int cols;
    
    LpaStarCell l;
    vector<LpaStarCell* > U; //Priority Queue
    LpaStarCell* start;
    LpaStarCell* goal;


    
    
public:
    
    //void initialise(int startX, int startY, int goalX, int goalY);
    double minValue(double g_, double rhs_);
    //double maxValue(double v1, double v2);
    int maxValue(int v1, int v2);
    void calcKey(int x, int y);
    void calcKey(LpaStarCell *cell);
    //void calc_H(int x, int y);
    double calc_H(int x, int y);
    void updateHValues();
    void updateAllKeyValues();
    
    //void copyMazeToDisplayMap(GridWorld gWorld);
    friend void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa);
};

#endif
