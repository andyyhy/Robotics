//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "steve";
robot.partner_name = "junnnli and andyyhy"

// initialize start pose of robot in the world
robot.origin = { xyz: [0, 2.5, 0], rpy: [0, 0, 0] };

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";

// specify and create data objects for the links of the robot
robot.links = { "base": {}, "left_arm": {}, "right_arm": {}, "left_leg": {}, "right_leg": {}, "head": {} };

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.left_arm_joint = { parent: "base", child: "left_arm" };
robot.joints.left_arm_joint.origin = { xyz: [-0.625, 0.625, 0.0], rpy: [0, 0, 0] };
robot.joints.left_arm_joint.axis = [-1.0, 0.0, 0];  // simpler axis 

robot.joints.right_arm_joint = { parent: "base", child: "right_arm" };
robot.joints.right_arm_joint.origin = { xyz: [0.625, 0.625, 0.0], rpy: [0, 0, 0] };
robot.joints.right_arm_joint.axis = [-1.0, 0.0, 0];  // simpler axis 

robot.joints.right_leg_joint = { parent: "base", child: "left_leg" };
robot.joints.right_leg_joint.origin = { xyz: [-0.625, -0.75, 0.0], rpy: [0, 0, 0] };
robot.joints.right_leg_joint.axis = [-1.0, 0.0, 0];  // simpler axis 

robot.joints.left_leg_joint = { parent: "base", child: "right_leg" };
robot.joints.left_leg_joint.origin = { xyz: [0.625, -0.75, 0.0], rpy: [0, 0, 0] };
robot.joints.left_leg_joint.axis = [-1.0, 0.0, 0];  // simpler axis 

robot.joints.head_joint = { parent: "base", child: "head" };
robot.joints.head_joint.origin = { xyz: [0, 0.75, 0.0], rpy: [0, 0, 0] };
robot.joints.head_joint.axis = [0, 1, 0];  // simpler axis 

/*
robot.joints.joint3 = { parent: "base", child: "head" };
//robot.joints.joint3.origin = {xyz: [0.5,0,0], rpy:[0,0,-1.57]};
robot.joints.joint3.origin = { xyz: [0.5, 0, 0], rpy: [0, 0, -Math.PI / 2] };
//robot.joints.joint3.axis = [0.707,-0.707,0];
robot.joints.joint3.axis = [Math.cos(Math.PI / 4), -Math.cos(Math.PI / 4), 0];
*/

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "head_joint";
robot.endeffector.position = [[0], [0], [0], [1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry(1.25, 1.5, 0.5);
links_geom["base"].applyMatrix(new THREE.Matrix4().makeTranslation(0, 0, 0));

links_geom["left_arm"] = new THREE.CubeGeometry(0.35, 1.5, 0.35);
links_geom["left_arm"].applyMatrix(new THREE.Matrix4().makeTranslation(-0.2, -0.75, 0));

links_geom["right_arm"] = new THREE.CubeGeometry(0.35, 1.5, 0.35);
links_geom["right_arm"].applyMatrix(new THREE.Matrix4().makeTranslation(0.2, -0.75, 0));

links_geom["right_leg"] = new THREE.CubeGeometry(0.625, 1.5, 0.5);
links_geom["right_leg"].applyMatrix(new THREE.Matrix4().makeTranslation(-0.3, -0.75, 0));

links_geom["left_leg"] = new THREE.CubeGeometry(0.625, 1.5, 0.5);
links_geom["left_leg"].applyMatrix(new THREE.Matrix4().makeTranslation(0.3, -0.75, 0));

links_geom["head"] = new THREE.CubeGeometry(0.75, 0.6, 0.6);
links_geom["head"].applyMatrix(new THREE.Matrix4().makeTranslation(0, 0.3, 0));