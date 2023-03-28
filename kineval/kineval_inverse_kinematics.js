
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik) || (kineval.params.persist_ik)) {
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial() {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime() - kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime() - kineval.params.trial_ik_random.start.getTime();

    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform, robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0] - endeffector_world[0][0], 2.0)
        + Math.pow(kineval.params.ik_target.position[1][0] - endeffector_world[1][0], 2.0)
        + Math.pow(kineval.params.ik_target.position[2][0] - endeffector_world[2][0], 2.0));

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2 * (Math.random() - 0.5);
        kineval.params.ik_target.position[1][0] = 1.2 * (Math.random() - 0.5) + 1.5;
        kineval.params.ik_target.position[2][0] = 0.7 * (Math.random() - 0.5) + 0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length


    //compute the base frame of endeffector position


    var endeffector_position_world = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    //console.log("endeffector_pos_local", endeffector_position_local);
    //console.log("endeffector_pos_world", endeffector_position_world);

    //Get all joint names
    var joint_names = [];
    var cur_joint = endeffector_joint;
    joint_names.push(cur_joint);
    console.log(robot.base)
    while (robot.joints[cur_joint].parent != robot.base) {
        cur_joint = robot.links[robot.joints[cur_joint].parent].parent;
        joint_names.push(cur_joint);
    }
    joint_names = joint_names.reverse();

    //Initialize jacobian
    var jacobian = Array(6);
    for (var i = 0; i < jacobian.length; i++) {
        jacobian[i] = Array(joint_names.length);
    }


    //Compute Jacobian  
    for (var i = 0; i < joint_names.length; i++) {

        //Calculate linear velocity Jacobian
        var joint_origin_world = matrix_multiply(robot.joints[joint_names[i]].xform, [[0], [0], [0], [1]]);
        var temp_axis = Array(4);
        temp_axis[0] = [robot.joints[joint_names[i]].axis[0]];
        temp_axis[1] = [robot.joints[joint_names[i]].axis[1]];
        temp_axis[2] = [robot.joints[joint_names[i]].axis[2]];
        temp_axis[3] = [1];

        var joint_axis_world = matrix_multiply(robot.joints[joint_names[i]].xform, temp_axis);

        var temp_cross_RHS = [];
        for (var j = 0; j < 3; j++) {
            temp_cross_RHS.push(endeffector_position_world[j][0] - joint_origin_world[j][0]);
        }
        var temp_cross_LHS = [];

        for (var j = 0; j < 3; j++) {
            temp_cross_LHS.push(joint_axis_world[j][0] - joint_origin_world[j][0]);
        }

        //console.log(temp)


        jacobian_linear = vector_cross(temp_cross_LHS, temp_cross_RHS);
        jacobian[0][i] = jacobian_linear[0];
        jacobian[1][i] = jacobian_linear[1];
        jacobian[2][i] = jacobian_linear[2];

        //Calculate angular velocity jacobian
        jacobian[3][i] = temp_cross_LHS[0];
        jacobian[4][i] = temp_cross_LHS[1];
        jacobian[5][i] = temp_cross_LHS[2];
    }


    //console.log(jacobian)


    //Find the error term (6x1 matrix)
    var dx = [
        [endeffector_target_world.position[0][0] - endeffector_position_world[0][0]],
        [endeffector_target_world.position[1][0] - endeffector_position_world[1][0]],
        [endeffector_target_world.position[2][0] - endeffector_position_world[2][0]],
        [0],
        [0],
        [0]
    ];

    //Compute joint config change term
    if (kineval.params.ik_pseudoinverse) {
        var jacobian_pseudoinverse = matrix_pseudoinverse(jacobian);
        var dq = matrix_multiply(jacobian_pseudoinverse, dx);
    }
    else {
        var jacobian_transpose = matrix_transpose(jacobian);
        var dq = matrix_multiply(jacobian_transpose, dx);
    }

    //Update each joint
    for (var i = 0; i < joint_names.length; i++) {
        robot.joints[joint_names[i]].control = kineval.params.ik_steplength * dq[i][0];
    }

    robot.dx = dx;
    robot.jacobian = jacobian;
    robot.dq = dq;

    //console.log(robot.dx);
    //console.log(robot.jacobian);
    //console.log(robot.dq);

}
