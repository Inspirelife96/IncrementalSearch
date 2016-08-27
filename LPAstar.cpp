#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow

#include "LPAstar.h"
#include "gridworld.h"


LpaStar::LpaStar() {
    
}

LpaStar::~LpaStar() {
    
}

void LpaStar::Initilize(GridWorld &gWorld) {
  
	int rowMax = gWorld.getGridWorldRows();
    int colMax = gWorld.getGridWorldCols();

	maze.resize(rowMax); 

    for (int row = 0; row < rowMax; row++) {
		maze[row].resize(colMax);
        for (int col = 0; col < colMax; col++) {
            maze[row][col].s_g = 0;
            maze[row][col].s_h = 0;
            maze[row][col].s_is_in_closetable = 0;
            maze[row][col].s_is_in_opentable = 0;
            maze[row][col].type = gWorld.getMapTypeValue(row, col);
            maze[row][col].x = col;
            maze[row][col].y = row;
            maze[row][col].parent = NULL;
            
            if (maze[row][col].type == '6' )  // 起点
            {
                start = &(maze[row][col]);
            }
            else if(maze[row][col].type == '7')    // 终点
            {
                goal = &(maze[row][col]);
            }
        }
    }

	rows = rowMax;
	cols = colMax;
	close_node_count = 0;
	open_node_count = 0;
	top = -1;

}

int LpaStar::Search(){
    open_table[open_node_count++] = start;         // 起始点加入open表
    
    start->s_is_in_opentable = 1;               // 加入open表
    start->s_g = 0;
    start->s_h = (abs(goal->x - start->x) + abs(goal->y - start->y))*10;
    start->parent = NULL;
    
    if ( start->x == goal->x && start->y == goal->y )
    {
        printf("起点==终点！\n");
        return 0;
    }
    
    int is_found = 0;
    LpaStarCell *curr_node = NULL;
    
    while( 1 )
    {
        curr_node = open_table[0];      // open表的第一个点一定是f值最小的点(通过堆排序得到的)
        open_table[0] = open_table[--open_node_count];  // 最后一个点放到第一个点，然后进行堆调整
        
        close_table[close_node_count++] = curr_node;    // 当前点加入close表
        curr_node->s_is_in_closetable = 1;       // 已经在close表中了
        
        if ( curr_node->x == goal->x && curr_node->y == goal->y )// 终点在close中，结束
        {
            is_found = 1;
            break;
        }
        
        get_neighbors( curr_node, goal );           // 对邻居的处理
        
        if ( open_node_count == 0 )             // 没有路径到达
        {
            is_found = 0;
            break;
        }

		adjust_heap( 0 );               // 调整堆
    }
    
    if ( is_found )
    {
        curr_node = goal;
        
        while( curr_node )
        {
            path_stack[++top] = curr_node;
            curr_node = curr_node->parent;
            if (curr_node != NULL && curr_node->type != '6' &&  curr_node->type != '7') {
                curr_node->type = '4';
            }
        }
        
        //while( top >= 0 )        // 下面是输出路径看看~
        //{
        //    if ( top > 0 )
        //    {
        //        printf("(%d,%d)-->", path_stack[top]->x, path_stack[top--]->y);
        //    }
        //    else
        //    {
        //       printf("(%d,%d)", path_stack[top]->x, path_stack[top--]->y);
        //    }
        //}
    }
    else
    {
        printf("么有找到路径");
    }
    
    puts("");
    
    return 0;
}



// 堆调整
//
void LpaStar::adjust_heap( int /*i*/nIndex )
{
    int curr = nIndex;
    int child = curr * 2 + 1;   // 得到左孩子idx( 下标从0开始，所有做孩子是curr*2+1 )
    int parent = ( curr - 1 ) / 2;  // 得到双亲idx
    
    if (nIndex < 0 || nIndex >= open_node_count)
    {
        return;
    }
    
    // 往下调整( 要比较左右孩子和cuur parent )
    //
    while ( child < open_node_count )
    {
        // 小根堆是双亲值小于孩子值
        //
        if ( child + 1 < open_node_count && open_table[child]->s_g + open_table[child]->s_h  > open_table[child+1]->s_g + open_table[child+1]->s_h )
        {
            ++child;// 判断左右孩子大小
        }
        
        if (open_table[curr]->s_g + open_table[curr]->s_h <= open_table[child]->s_g + open_table[child]->s_h)
        {
            break;
        }
        else
        {
            swap( child, curr );            // 交换节点
            curr = child;               // 再判断当前孩子节点
            child = curr * 2 + 1;           // 再判断左孩子
        }
    }
    
    if (curr != nIndex)
    {
        return;
    }
    
    // 往上调整( 只需要比较cuur child和parent )
    //
    while (curr != 0)
    {
        if (open_table[curr]->s_g + open_table[curr]->s_h >= open_table[parent]->s_g + open_table[parent]->s_h)
        {
            break;
        }
        else
        {
            swap( curr, parent );
            curr = parent;
            parent = (curr-1)/2;
        }
    }
}

// 查找邻居
// 对上下左右8个邻居进行查找
//
void LpaStar::get_neighbors( LpaStarCell *curr_node, LpaStarCell *end_node )
{
    int x = curr_node->x;
    int y = curr_node->y;
    
    // 下面对于8个邻居进行处理！
    //
    if ( ( x + 1 ) >= 0 && ( x + 1 ) < cols && y >= 0 && y < rows )
    {
        insert_to_opentable( x+1, y, curr_node, end_node, 10 );
    }
    
    if ( ( x - 1 ) >= 0 && ( x - 1 ) < cols && y >= 0 && y < rows )
    {
        insert_to_opentable( x-1, y, curr_node, end_node, 10 );
    }
    
    if ( x >= 0 && x < cols && ( y + 1 ) >= 0 && ( y + 1 ) < rows )
    {
        insert_to_opentable( x, y+1, curr_node, end_node, 10 );
    }
    
    if ( x >= 0 && x < cols && ( y - 1 ) >= 0 && ( y - 1 ) < rows )
    {
        insert_to_opentable( x, y-1, curr_node, end_node, 10 );
    }
    
    if ( ( x + 1 ) >= 0 && ( x + 1 ) < cols && ( y + 1 ) >= 0 && ( y + 1 ) < rows )
    {
        insert_to_opentable( x+1, y+1, curr_node, end_node, 14 );
    }
    
    if ( ( x + 1 ) >= 0 && ( x + 1 ) < cols && ( y - 1 ) >= 0 && ( y - 1 ) < rows )
    {
        insert_to_opentable( x+1, y-1, curr_node, end_node, 14 );
    }
    
    if ( ( x - 1 ) >= 0 && ( x - 1 ) < cols && ( y + 1 ) >= 0 && ( y + 1 ) < rows )
    {
        insert_to_opentable( x-1, y+1, curr_node, end_node, 14 );
    }
    
    if ( ( x - 1 ) >= 0 && ( x - 1 ) < cols && ( y - 1 ) >= 0 && ( y - 1 ) < rows )
    {
        insert_to_opentable( x-1, y-1, curr_node, end_node, 14 );
    }
}

// 判断邻居点是否可以进入open表
//
void LpaStar::insert_to_opentable( int x, int y, LpaStarCell *curr_node, LpaStarCell *end_node, int w )
{
    int i;
    
    if (maze[y][x].type != '1' )        // 不是障碍物
    {
        if ( !maze[y][x].s_is_in_closetable )   // 不在闭表中
        {
            if ( maze[y][x].s_is_in_opentable ) // 在open表中
            {
                // 需要判断是否是一条更优化的路径
                //
                if ( maze[y][x].s_g > curr_node->s_g + w )    // 如果更优化
                {
                    maze[y][x].s_g = curr_node->s_g + w;
                    maze[y][x].parent = curr_node;
                    
                    for ( i = 0; i < open_node_count; ++i )
                    {
                        if (open_table[i]->x == maze[y][x].x && open_table[i]->y == maze[y][x].y )
                        {
                            break;
                        }
                    }
                    
                    adjust_heap(i);                   // 下面调整点
                }
            }
            else                                    // 不在open中
            {
                maze[y][x].s_g = curr_node->s_g + w;
                maze[y][x].s_h = (abs(end_node->x - x ) + abs(end_node->y - y))*10;
                maze[y][x].parent = curr_node;
                maze[y][x].s_is_in_opentable = 1;
                open_table[open_node_count++] = &(maze[y][x]);
            }
        }
    }
}

// 交换两个元素
//
void LpaStar::swap( int idx1, int idx2 )
{
    LpaStarCell *tmp = open_table[idx1];
    open_table[idx1] = open_table[idx2];
    open_table[idx2] = tmp;
}

double LpaStar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int LpaStar::maxValue(int v1, int v2){
	
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

double LpaStar::calc_H(int x, int y){
	
	int diffY = abs(goal->y - y);
	int diffX = abs(goal->x - x);
	
	//maze[y][x].h = (double)maxValue(diffY, diffX);
	return (double)maxValue(diffY, diffX);
}

void LpaStar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
	
	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

void LpaStar::calcKey(int x, int y){
	double key1, key2;
	
	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h;
}


void LpaStar::calcKey(LpaStarCell *cell){
	double key1, key2;
	
	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + cell->h;
	
	cell->key[0] = key1;
	cell->key[1] = key2;
}

void LpaStar::updateAllKeyValues(){	
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   calcKey(&maze[i][j]);
		}
	}
	
	calcKey(start);
	calcKey(goal);
}




