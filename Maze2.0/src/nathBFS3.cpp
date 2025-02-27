#include <iostream>
#include <vector>
#include <queue>

using namespace std;

#define ROWS 5
#define COLS 5

// Structure to represent a cell in the maze
struct Cell 
{
    int row, col;
};

// Class for BFS queue
class MazeSolver 
{
private:
    vector<vector<bool> > visited;
    vector<vector<Cell> > came_from;
    vector<vector<char> > maze;
    Cell start, end;

public:
    MazeSolver(vector<vector<char> > maze, Cell start, Cell end) : maze(maze), start(start), end(end) 
    {
        visited.resize(ROWS, vector<bool>(COLS, false));
        came_from.resize(ROWS, vector<Cell>(COLS, {-1, -1}));
    }

    // Function to check if a cell is valid
    bool isValid(int r, int c) 
    {
        return (r >= 0 && r < ROWS && c >= 0 && c < COLS && maze[r][c] != '#' && !visited[r][c]);
    }

    // Function to reconstruct the path
    void reconstructPath() 
    {
        vector<Cell> path;
        Cell current = end;

        while (current.row != start.row || current.col != start.col) 
        {
            path.push_back(current);
            current = came_from[current.row][current.col];

            if (path.size() > ROWS * COLS) 
            {
                cout << "Infinite loop detected." << endl;
                exit(1);
            }
        }

        path.push_back(start); // Add start cell
        reverse(path.begin(), path.end()); // Reverse the path

        cout << "Path found:\n";
        for (const auto& cell : path) 
        {
            cout << "(" << cell.row << ", " << cell.col << ") ";
            if (maze[cell.row][cell.col] != 'S' && maze[cell.row][cell.col] != 'E') 
            {
                maze[cell.row][cell.col] = '*'; // Mark path on the maze
            }
        }
        cout << endl;

        // Print the maze with the path marked
        for (const auto& row : maze) 
        {
            for (char cell : row) 
            {
                cout << cell;
            }
            cout << endl;
        }
    }

    // BFS function to solve the maze
    bool solve() 
    {
        queue<Cell> q;
        q.push(start);
        visited[start.row][start.col] = true;
        came_from[start.row][start.col] = {-1, -1}; // Mark start

        // Possible moves (up, down, left, right)
        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!q.empty()) 
        {
            Cell current = q.front();
            q.pop();

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
                    q.push({nr, nc});
                    visited[nr][nc] = true;
                    came_from[nr][nc] = current;
                }
            }
        }
        return false; // No path found
    }
};

int main() 
{
    auto start_time = std::chrono::high_resolution_clock::now();

    vector<vector<char> > maze = 
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
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the elapsed time
    std::chrono::duration<double> elapsed_time = end_time - start_time;

    // Display the execution time
    std::cout << "BFS Execution Time: " << elapsed_time.count() << " seconds" << std::endl;

    return 0;
}
