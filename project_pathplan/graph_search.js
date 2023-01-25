/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {

    // create the search queue
    visit_queue = [];
    goal_node = null;
    start_node = null;

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind = 0, xpos = -2; xpos < 7; iind++, xpos += eps) {
        G[iind] = [];
        for (jind = 0, ypos = -2; ypos < 7; jind++, ypos += eps) {
            G[iind][jind] = {
                i: iind, j: jind, // mapping to graph array
                x: xpos.toFixed(10), y: ypos.toFixed(10), // mapping to map coordinates
                parent: null, // pointer to parent in graph along motion path
                distance: 10000, // distance to start via path through parent
                visited: false, // flag for whether the node has been visited
                priority: null, // visit priority based on fscore
                queued: false // flag for whether the node has been queued for visiting
            };


            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            if (xpos.toFixed(5) == q_init[0] && ypos.toFixed(5) == q_init[1]) {

                start_node = G[iind][jind];
            }
            if (xpos.toFixed(5) == q_goal[0] && ypos.toFixed(5) == q_goal[1]) {
                goal_node = G[iind][jind];
            }

        }
    }

    start_node.queued = true;
    start_node.distance = 0;
    start_node.priority = start_node.distance + Math.sqrt(
        Math.pow((goal_node.i - start_node.i), 2) +
        Math.pow((goal_node.j - start_node.j), 2));
    minheaper.insert(visit_queue, start_node);
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    // Check if visite queue is empty. If empty, the search has failed
    if (visit_queue.length == 0) {
        search_iterate = false;
        return "failed";
    }

    // Pop a node from pq and set visited to true
    var current_node = minheaper.extract(visit_queue);
    current_node.visited = true;
    draw_2D_configuration([current_node.x, current_node.y], "visited")
    search_visited++;

    //Check if current node is an obstacle
    if (testCollision([current_node.x, current_node.y]) == true) {
        return "iterating";
    }

    //Check if the node is the goal node
    if (current_node.x == q_goal[0] && current_node.y == q_goal[1]) {
        search_iterate = false;
        drawHighlightedPathGraph(current_node)
        return "succeeded";
    }

    //Find all neighbor nodes
    var neighbors = []
    if (current_node.i > 0) {
        neighbors.push(G[current_node.i - 1][current_node.j]);
    }
    if (current_node.i < 800) {
        neighbors.push(G[current_node.i + 1][current_node.j]);
    }
    if (current_node.j > 0) {
        neighbors.push(G[current_node.i][current_node.j - 1]);
    }
    if (current_node.j < 800) {
        neighbors.push(G[current_node.i][current_node.j + 1]);
    }
    //Add appropriate neighbors to search queue
    for (var neighbor of neighbors) {
        //Check if neighbor has been visited or queued
        if (neighbor.visited == true || neighbor.queued == true) {
            continue;
        }
        if (neighbor.distance > current_node.distance + eps) {
            neighbor.parent = current_node;
            neighbor.distance = (current_node.distance + eps);
            neighbor.queued = true;
            neighbor.priority = neighbor.distance + Math.sqrt(
                Math.pow((goal_node.x - neighbor.x), 2) +
                Math.pow((goal_node.y - neighbor.y), 2));
            minheaper.insert(visit_queue, neighbor);
            console.log(neighbor.priority)
            draw_2D_configuration([neighbor.x, neighbor.y], "queued")
        }
    }
    return "iterating";

}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// STENCIL: implement min heap functions for graph search priority queue.
//   These functions work use the 'priority' field for elements in graph.

minheaper = {}

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    var element_index = heap.length;
    var parent_index = Math.floor((element_index - 1) / 2);
    heap.push(new_element);
    var done = (element_index <= 0) || (heap[parent_index].priority <= heap[element_index].priority);

    while (!done) {
        var temp = heap[parent_index];
        heap[parent_index] = heap[element_index];
        heap[element_index] = temp;

        element_index = parent_index;
        parent_index = Math.floor((element_index - 1) / 2);

        done = (element_index <= 0) || (heap[parent_index].priority <= heap[element_index].priority);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // STENCIL: implement your min binary heap extract operation
    var ans = heap[0];
    heap[0] = heap[heap.length - 1];
    heap.pop()
    var k = 1;
    while (2 * k <= heap.length) {
        var j = 2 * k;
        if (j < heap.length && heap[j - 1].priority > heap[j].priority) {
            j++;
        }
        if (heap[k - 1].priority <= heap[j - 1].priority) {
            break;
        }
        var temp = heap[k - 1];
        heap[k - 1] = heap[j - 1];
        heap[j - 1] = temp;
        k = j;
    }
    return ans;

}

// assign extract function within minheaper object

// STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;

