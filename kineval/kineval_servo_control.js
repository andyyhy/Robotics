
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return;

    // STENCIL: implement FSM to cycle through dance pose setpoints

    const current_index = kineval.params.dance_pose_index
    kineval.params.setpoint_target = kineval.setpoints[kineval.params.dance_sequence_index[current_index]];

    var done = true;
    for (x in robot.joints) {
        var servo_error = Math.abs(kineval.params.setpoint_target[x] - robot.joints[x].angle);
        if (servo_error > 0.001) {
            done = false;
            break;
        }
    }

    if (done) {
        kineval.params.dance_pose_index = (kineval.params.dance_pose_index + 1) % kineval.params.dance_sequence_index.length;
    }


}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return;

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds() / 60 * 2 * Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control() {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd) && (!kineval.params.persist_pd)) return;

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    var x;
    for (x in robot.joints) {
        var servo_error = kineval.params.setpoint_target[x] - robot.joints[x].angle;
        robot.joints[x].control = servo_error * robot.joints[x].servo.p_gain;
    }
}
