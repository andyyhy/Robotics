
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal || kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index + 1) % kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


// STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i;
    q_goal_config = new Array(q_start_config.length);
    for (i = 0; i < q_goal_config.length; i++) q_goal_config[i] = 0;

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();

    T_a = tree_init(q_start_config);

    T_b = tree_init(q_goal_config);

    kineval.motion_plan = [];
}



function robot_rrt_planner_iterate() {

    var i;
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now() - cur_time > 10)) {
        cur_time = Date.now();

        // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
        //   is used instead of a for loop to avoid blocking and non-responsiveness 
        //   in the browser.
        //
        //   once plan is found, highlight vertices of found path by:
        //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
        //
        //   provided support functions:
        //
        //   kineval.poseIsCollision - returns if a configuration is in collision
        //   tree_init - creates a tree of configurations
        //   tree_add_vertex - adds and displays new configuration vertex for a tree
        //   tree_add_edge - adds and displays new tree edge between configurations

        q_random = random_config();
        if (rrt_extend(T_a, q_random) != "trapped") {
            if (rrt_connect(T_b, T_a.vertices[T_a.newest].vertex) == "reached") {
                var T_a_path = find_path(T_a);
                var T_b_path = find_path(T_b);
                T_a_path.reverse();
                var path = generate_path_tree(T_a_path, T_b_path);
                search_iterate = false;
                return "reached";
            }
        }
        swap(T_a, T_b);
        return "extended";

    }

}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree, q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1, 0.1, 0.1);
    temp_material = new THREE.MeshLambertMaterial({ color: 0xffff00, transparent: true, opacity: 0.7 });
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree, q1_idx, q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}


//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


// STENCIL: implement RRT-Connect functions here, such as:
//   rrt_extend
//   rrt_connect
//   random_config
//   new_config
//   nearest_neighbor
//   normalize_joint_state
//   find_path
//   path_dfs



function rrt_extend(tree, q) {
    var q_nearest_index = nearest_neighbor(tree, q);
    //console.log(q_nearest_index);
    var new_config = find_new_config(q, tree.vertices[q_nearest_index].vertex);

    if (kineval.poseIsCollision(new_config)) {
        return "trapped";
    }

    tree_add_vertex(tree, new_config);
    tree_add_edge(tree, q_nearest_index, tree.newest);
    tree.vertices[tree.newest].parent = q_nearest_index;

    if (find_distance(new_config, q) < 0.3) {
        return "reached";
    }
    return "advanced";
}

function rrt_connect(tree, q) {
    do {
        var status = rrt_extend(tree, q);
    } while (status == "advanced")

    return status;
}

function random_config() {


    var x = Math.random() * (robot_boundary[1][0] - robot_boundary[0][0]) + robot_boundary[0][0];
    var y = 0;
    var z = Math.random() * (robot_boundary[1][2] - robot_boundary[0][2]) + robot_boundary[0][2];

    var roll = 0;
    var pitch = Math.random() * (2 * Math.PI);
    var yaw = 0;

    var random_config = [
        x,
        y,
        z,
        roll,
        pitch,
        yaw
    ]

    for (x in robot.joints) {
        random_config = random_config.concat(Math.random() * (2 * Math.PI));
    }

    return random_config;
}

function find_new_config(q, q_nearest) {

    var distance = find_distance(q, q_nearest);

    if (distance < 0.3) {
        return q;
    }

    var t = 0.3 / distance;

    var new_q_x = (1 - t) * q_nearest[0] + t * q[0];
    var new_q_z = (1 - t) * q_nearest[2] + t * q[2];


    var new_config = { ...q };

    new_config[0] = new_q_x;
    new_config[2] = new_q_z;


    return new_config;
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
    var dz = q2[2] - q1[2];
    var distance = Math.sqrt(dx * dx + dz * dz);
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

    for (var i = 0; i < path_a.length; i++) {
        T_a.vertices[path_a[i]].geom.material.color = { r: 1, g: 0, b: 0 };
        kineval.motion_plan.push(T_a.vertices[path_a[i]]);
    }
    for (var i = 0; i < path_b.length; i++) {
        T_b.vertices[path_b[i]].geom.material.color = { r: 1, g: 0, b: 0 };
        kineval.motion_plan.push(T_b.vertices[path_b[i]]);
    }

}

function swap(T_1, T_2) {
    var temp = { ...T_1 };

    Object.assign(T_1, T_2);
    Object.assign(T_2, temp);
}


