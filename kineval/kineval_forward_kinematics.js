
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics() {

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: call kineval.buildFKTransforms();
    kineval.buildFKTransforms();

}

// STENCIL: implement buildFKTransforms, which kicks off
//   a recursive traversal over links and
//   joints starting from base, using following functions:
//     traverseFKBase
//     traverseFKLink
//     traverseFKJoint

kineval.buildFKTransforms = function buildFKTransforms() {
    var mstack = generate_identity();
    kineval.traverseFKBase(mstack);
}

kineval.traverseFKBase = function traverseFKBase(mstack) {
    var temp = matrix_multiply(mstack, generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]));
    temp = matrix_multiply(temp, generate_rotation_matrix_Z(robot.origin.rpy[2]));
    temp = matrix_multiply(temp, generate_rotation_matrix_Y(robot.origin.rpy[1]));
    mstack = matrix_multiply(temp, generate_rotation_matrix_X(robot.origin.rpy[0]));
    robot.origin.xform = matrix_copy(mstack);
    robot.links[robot.base].xform = matrix_copy(mstack);
    for (var i = 0; i < robot.links[robot.base].children.length; i++) {
        this.traverseFKJoint(robot.links[robot.base].children[i], mstack);
    }

    robot_heading = matrix_multiply(robot.origin.xform, [[0], [0], [1], [1]]);
    robot_lateral = matrix_multiply(robot.origin.xform, [[1], [0], [0], [1]]);
}

kineval.traverseFKJoint = function traverseFKJoint(cur, mstack) {
    var temp = matrix_multiply(mstack, generate_translation_matrix(robot.joints[cur].origin.xyz[0], robot.joints[cur].origin.xyz[1], robot.joints[cur].origin.xyz[2]));
    temp = matrix_multiply(temp, generate_rotation_matrix_Z(robot.joints[cur].origin.rpy[2]));
    temp = matrix_multiply(temp, generate_rotation_matrix_Y(robot.joints[cur].origin.rpy[1]));
    temp = matrix_multiply(temp, generate_rotation_matrix_X(robot.joints[cur].origin.rpy[0]));
    q = kineval.quaternionFromAxisAngle(robot.joints[cur].axis, robot.joints[cur].angle);
    q_mat = kineval.quaternionToRotationMatrix(q);
    mstack = matrix_multiply(temp, q_mat);
    robot.joints[cur].xform = matrix_copy(mstack);
    this.traverseFKLink(robot.links[robot.joints[cur].child].name, mstack);

}

kineval.traverseFKLink = function traverseFKLink(cur, mstack) {
    robot.links[cur].xform = matrix_copy(mstack);
    if (!robot.links[cur].children) {
        return;
    }
    else {
        for (var i = 0; i < robot.links[cur].children.length; i++) {
            this.traverseFKJoint(robot.links[cur].children[i], mstack);
        }
    }
}
    //
    // To use the keyboard interface, assign the global variables
    //   "robot_heading" and "robot_lateral",
    //   which represent the z-axis (heading) and x-axis (lateral)
    //   of the robot's base in its own reference frame,
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form
    //   as 4x1 homogenous matrices


    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

