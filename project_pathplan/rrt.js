/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

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

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    q_random = random_config();
    //console.log(q_random);
    if (rrt_extend(T_a, q_random) != "trapped") {
        if (rrt_connect(T_b, T_a.vertices[T_a.newest].vertex) == "reached") {
            var T_a_path = find_path(T_a);
            var T_b_path = find_path(T_b);
            T_a_path.reverse();
            var path = generate_path_tree(T_a_path, T_b_path);
            console.log(path);
            drawHighlightedPath(path.vertices);
            search_iterate = false;
            return "succeeded";
        }
    }
    swap(T_a, T_b);
    return "extended";




}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// STENCIL: implement RRT-Connect functions here, such as:
//   extendRRT
//   connectRRT
//   randomConfig
//   newConfig
//   findNearestNeighbor
//   dfsPath

function rrt_extend(tree, q) {
    var q_nearest_index = nearest_neighbor(tree, q);
    //console.log(q_nearest_index);
    var new_config = find_new_config(q, tree.vertices[q_nearest_index].vertex);

    if (testCollision(new_config)) {
        return "trapped";
    }

    insertTreeVertex(tree, new_config);
    insertTreeEdge(tree, q_nearest_index, tree.newest);
    tree.vertices[tree.newest].parent = q_nearest_index;

    if (find_distance(new_config, q) < eps) {
        return "reached";
    }
    search_visited++;
    return "advanced";
}

function rrt_connect(tree, q) {
    do {
        var status = rrt_extend(tree, q);
    } while (status == "advanced")

    return status;
}

function random_config() {
    var x = Math.random() * (7 + 2) - 2;
    var y = Math.random() * (7 + 2) - 2;
    return [x, y];
}

function find_new_config(q, q_nearest) {

    var distance = find_distance(q, q_nearest);

    if (distance < eps) {
        return q;
    }

    var step_x = (q[0] - q_nearest[0]) / distance;
    var step_y = (q[1] - q_nearest[1]) / distance;

    var new_q_x = q_nearest[0] + step_x * eps;
    var new_q_y = q_nearest[1] + step_y * eps;

    return [new_q_x, new_q_y];
}

function nearest_neighbor(tree, q) {
    var minDistance = Infinity;
    var q_nearest = null;
    var q_nearest_index = null;

    for (var i = 0; i < tree.vertices.length; i++) {
        var q_cur = tree.vertices[i].vertex;
        var distance = find_distance(q_cur, q)

        if (distance < minDistance) {
            minDistance = distance;
            q_nearest = q_cur;
            q_nearest_index = i;
        }
    }
    return q_nearest_index;
}

function find_distance(q1, q2) {
    var dx = q2[0] - q1[0];
    var dy = q2[1] - q1[1];
    var distance = Math.sqrt(dx * dx + dy * dy);
    return distance;
}

function find_path(tree) {
    var path = [tree.newest];
    var cur_vertex_index = tree.newest;

    while (cur_vertex_index !== 0) {
        var parent_vertex_index = tree.vertices[cur_vertex_index].parent;
        path.push(parent_vertex_index);
        cur_vertex_index = parent_vertex_index;
    }

    return path;
}
function generate_path_tree(path_a, path_b) {
    var T_path = initRRT(q_init);

    for (var i = 0; i < path_a.length; i++) {
        insertTreeVertex(T_path, T_a.vertices[path_a[i]].vertex);

    }
    for (var i = 0; i < path_b.length; i++) {
        insertTreeVertex(T_path, T_b.vertices[path_b[i]].vertex);
    }

    T_path.vertices.shift();
    return T_path;

}

function swap(T_1, T_2) {
    var temp = { ...T_1 };

    Object.assign(T_1, T_2);
    Object.assign(T_2, temp);
}