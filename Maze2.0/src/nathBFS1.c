#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define ROWS 5
#define COLS 5

// Structure to represent a cell in the maze
typedef struct {
    int row;
    int col;
} Cell;

// Structure to represent a node in the queue
typedef struct QueueNode {
    Cell cell;
    struct QueueNode* next;
} QueueNode;

// Structure for the queue
typedef struct {
    QueueNode* front;
    QueueNode* rear;
} Queue;

// Function to create a new queue
Queue* createQueue() {
    Queue* q = (Queue*)malloc(sizeof(Queue));
    if (!q) {
        perror("Failed to allocate memory for queue");
        exit(EXIT_FAILURE);
    }
    q->front = q->rear = NULL;
    return q;
}

// Function to check if the queue is empty
bool isEmpty(Queue* q) {
    return q->front == NULL;
}

// Function to enqueue a cell
void enqueue(Queue* q, Cell cell) {
    QueueNode* newNode = (QueueNode*)malloc(sizeof(QueueNode));
    if (!newNode) {
        perror("Failed to allocate memory for queue node");
        exit(EXIT_FAILURE);
    }
    newNode->cell = cell;
    newNode->next = NULL;

    if (isEmpty(q)) {
        q->front = q->rear = newNode;
    } else {
        q->rear->next = newNode;
        q->rear = newNode;
    }
}

// Function to dequeue a cell
Cell dequeue(Queue* q) {
    if (isEmpty(q)) {
        fprintf(stderr, "Error: Attempting to dequeue from an empty queue\n");
        exit(EXIT_FAILURE); // Or handle the error as appropriate
    }

    QueueNode* temp = q->front;
    Cell cell = temp->cell;
    q->front = q->front->next;

    if (q->front == NULL) {
        q->rear = NULL; // Queue is now empty
    }

    free(temp);
    return cell;
}

// Function to free the queue
void freeQueue(Queue* q) {
    while (!isEmpty(q)) {
        dequeue(q);
    }
    free(q);
}

// Function to check if a cell is valid (within bounds and not a wall)
bool isValid(int rows, int cols, char maze[ROWS][COLS], int r, int c, bool visited[ROWS][COLS]) {
    return (r >= 0 && r < rows && c >= 0 && c < cols && maze[r][c] != '#' && !visited[r][c]);
}

// Function to reconstruct the path
void reconstructPath(Cell came_from[5][5], Cell start, Cell end, Cell path[], int *path_len) { // Fixed dimensions for simplicity
    Cell current = end;
    *path_len = 0;

    while (current.row != start.row || current.col != start.col) {
        path[(*path_len)++] = current;
        current = came_from[current.row][current.col];
        if(*path_len > 25)
        {
            printf("Infinite loop detected.\n"); //Just to prevent unexpected infinite loop
            exit(1);
        }
    }
    path[(*path_len)++] = start; // Add start cell.
   
    // Reverse the path
    for (int i = 0; i < *path_len / 2; i++) {
        Cell temp = path[i];
        path[i] = path[*path_len - 1 - i];
        path[*path_len - 1 - i] = temp;
    }
}

// BFS function to solve the maze
bool solveMaze(int rows, int cols, char maze[ROWS][COLS], Cell start, Cell end) {
    Queue* q = createQueue();
    bool visited[ROWS][COLS];  // Keep track of visited cells
    Cell came_from[ROWS][COLS]; //Keep track of where you came from

    // Initialize visited array
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            visited[i][j] = false;
        }
    }

    // Enqueue the starting cell and mark it as visited
    enqueue(q, start);
    visited[start.row][start.col] = true;
    came_from[start.row][start.col] = (Cell){-1, -1}; // Indicate start

    while (!isEmpty(q)) {
        Cell current = dequeue(q);

        // Check if we've reached the end
        if (current.row == end.row && current.col == end.col) {
            
            Cell path[rows * cols]; // Maximum possible path length
            int path_len;
            reconstructPath(came_from, start, end, path, &path_len);
            printf("Path found:\n");
            for (int i = 0; i < path_len; i++) {
                printf("(%d, %d) ", path[i].row, path[i].col);
                 if (maze[path[i].row][path[i].col] != 'S' && maze[path[i].row][path[i].col] != 'E') {
                    maze[path[i].row][path[i].col] = '*'; // Mark on the maze
                }
            }
            printf("\n");

             // Print the maze with the path marked
            for (int i = 0; i < rows; i++) {
                for (int j = 0; j < cols; j++) {
                    printf("%c", maze[i][j]);
                }
                printf("\n");
            }
            freeQueue(q);
            return true; // Path found
        }

        // Explore neighbors (up, down, left, right)
        int dr[] = {-1, 1, 0, 0};  // Row offsets
        int dc[] = {0, 0, -1, 1};  // Column offsets

        for (int i = 0; i < 4; i++) {
            int nr = current.row + dr[i];
            int nc = current.col + dc[i];

            if (isValid(rows, cols, maze, nr, nc, visited)) {
                Cell next_cell = {nr, nc};
                enqueue(q, next_cell);
                visited[nr][nc] = true;
                came_from[nr][nc] = current;
            }
        }
    }
    freeQueue(q);
    return false; // No path found
}

int main() {
    // Example maze (same as before, but as a 2D char array)
    char maze[ROWS][COLS] = {
        {'S', '.', '.', '.', '#'},
        {'.', '.', '.', '.', '#'},
        {'.', '#', '.', '.', '.'},
        {'#', '.', '.', '.', '.'},
        {'.', '.', '.', '#', 'E'}
    };
    int rows = ROWS;
    int cols = COLS;

    Cell start = {0, 0}; // Starting cell
    Cell end = {4, 4};   // Ending cell

    if (!solveMaze(rows, cols, maze, start, end)) {
        printf("No path found.\n");
    }

    return 0;
}