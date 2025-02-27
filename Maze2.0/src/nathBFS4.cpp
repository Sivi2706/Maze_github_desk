#include <iostream>
#include <queue>
#include <chrono>

using namespace std;

#define ROWS 5
#define COLS 5

struct Cell 
{
    int row, col;
};

class MazeSolver 
{
private:
    // declare 2D arrays instead of using std::vector type
    // std::vector uses heap allocation -> slows down memory access
    bool visited[ROWS][COLS] = {false};
    Cell came_from[ROWS][COLS];
    char maze[ROWS][COLS];      
    Cell start, end;

public:
    MazeSolver(const char inputMaze[ROWS][COLS], Cell start, Cell end) : start(start), end(end) 
    {
        // init the maze
        // avoid nested for loops
        std::memcpy(maze, inputMaze, sizeof(maze));
        std::fill(&came_from[0][0], &came_from[0][0] + ROWS * COLS, Cell{-1, -1});

        /*
        for (int i = 0; i < ROWS; i++) 
        {
            for (int j = 0; j < COLS; j++) 
            {
                maze[i][j] = inputMaze[i][j];
                came_from[i][j] = {-1, -1};
            }
        }
        */
    }

    // inline -> copy the function's code to where it was called
    // faster for short funcs compared to calling the whole thing
    inline bool isValid(int r, int c) 
    {
        return (r >= 0 && r < ROWS && c >= 0 && c < COLS && maze[r][c] != '#' && !visited[r][c]);
    }

    void reconstructPath() 
    {
        /*
        array size for path logic: 
        if every cewll is visited once, maximum queue size of ROWS * COLS.
        ========== EXAMPLE ==========
             ROWS
        COLS         0  1  2  3  4
                 --------------------
              0  |   0  1  2  3  4
              1  |   5  6  7  8  9
              2  |   10 11 12 13 14
              3  |   15 16 17 18 19
              4  |   20 21 22 23 24

        index mapping: (r, c) → r * COLS + c
        eg. (2, 3) → 2 * 5 + 3 = 13
        eg. (4, 2) → 4 * 5 + 2 = 22

        ========== TO REVERSE THE MAPPING ==========
                int row = index / COLS
                int col = index % COLS
         */
        Cell path[ROWS * COLS];
        int pathSize = 0;
        Cell current = end;

        while (current.row != start.row || current.col != start.col) 
        {
            path[pathSize++] = current;
            current = came_from[current.row][current.col];

            if (pathSize > ROWS * COLS) 
            {
                cout << "Infinite loop detected." << endl;
                exit(1);
            }
        }

        path[pathSize++] = start;

        cout << "Path found:\n";
        // reverse order
        for (int i = pathSize - 1; i >= 0; i--)
        {
            cout << "(" << path[i].row << ", " << path[i].col << ") ";
            if (maze[path[i].row][path[i].col] != 'S' && maze[path[i].row][path[i].col] != 'E') 
            {
                maze[path[i].row][path[i].col] = '*'; // Mark path on the maze
            }
        }
        cout << endl;

        // Print the maze with the path marked
        for (int i = 0; i < ROWS; i++) 
        {
            for (int j = 0; j < COLS; j++) 
            {
                cout << maze[i][j];
            }
            cout << endl;
        }
    }

    bool solve() 
    {
        Cell queue[ROWS * COLS];
        int front = 0, rear = 0;
        
        queue[rear++] = start;
        visited[start.row][start.col] = true;
        came_from[start.row][start.col] = {-1, -1}; // Mark start

        // Possible moves (up, down, left, right)
        const int dr[] = {-1, 1, 0, 0};
        const int dc[] = {0, 0, -1, 1};

        while (front < rear)
        {
            Cell current = queue[front++];

            if (current.row == end.row && current.col == end.col) 
            {
                reconstructPath();
                return true;
            }

            for (int i = 0; i < 4; i++) 
            {
                int nr = current.row + dr[i];
                int nc = current.col + dc[i];

                if (isValid(nr, nc)) 
                {
                    queue[rear++] = {nr, nc};
                    visited[nr][nc] = true;
                    came_from[nr][nc] = current;
                }
            }
        }
        return false;
    }
};

int main() 
{
    auto start_time = chrono::high_resolution_clock::now();

    // Define the maze using a static 2D array
    char maze[ROWS][COLS] = 
    {
        {'S', '.', '.', '.', '#'},
        {'.', '.', '.', '.', '#'},
        {'.', '#', '.', '.', '.'},
        {'#', '.', '.', '.', '.'},
        {'.', '.', '.', '#', 'E'}
    };

    Cell start = {0, 0};
    Cell end = {4, 4};

    MazeSolver solver(maze, start, end);

    if (!solver.solve()) 
    {
        cout << "No path found.\n";
    }

    // Stop the timer
    auto end_time = chrono::high_resolution_clock::now();

    // Calculate the elapsed time
    chrono::duration<double> elapsed_time = end_time - start_time;

    // Display the execution time
    cout << "BFS Execution Time: " << elapsed_time.count() << " seconds" << endl;
    // fastest: 3.1583e-05 seconds

    return 0;
}
