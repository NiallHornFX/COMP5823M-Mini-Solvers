#ifndef IK_SOLVER_H
#define IK_SOLVER_H

// Std Headers
#include <string>
#include <vector>

// Project Headers
#include "bvhdata.h"
#include "effector.h"
#include "anim_state.h"

// Ext Headers
// GLM
#include "ext/glm/glm.hpp"
// Eigen
#include "ext/Eigen/Eigen"
#include "ext/Eigen/Core"

// Class that handles the Inverse Kinematics Solve given some input joints and effectors. 

class IK_Solver
{
public:
	IK_Solver(Anim_State *Anim, const std::vector<Joint*> &joint_chain, const Joint *joint_end, const Effector &target);
	~IK_Solver() = default;

	//============== Tick / Update ===============
	void tick(); 

	// =============== Build Jacobian ===============
	void                   build_jacobian();
	std::vector<glm::vec3> perturb_joints();
	void                   perturb_traverse(Joint *perturb_joint, ChannelEnum dof, double perturb_factor);


	// =============== Solve for Angles ===============
	// Pseudo-Inverse Jacobian solve for Joint Angles

	// =============== Integrate Angles ===============

	// =============== Utility Static Member Functions ===============
	static Eigen::Vector3f glmToeig(const glm::vec3 &vec);
	static Eigen::Vector4f glmToeig(const glm::vec4 &vec);

public:
	std::vector<Joint*> chain; 
	Joint *end_joint; // End Effector / End_Site joint.
	Effector effector_target; 

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> *J; // Jacobian Matrix
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> *JI; // Jacobian Inverse 
	Eigen::Matrix<double, Eigen::Dynamic, 1> *V;  // Velocity Vector
	Eigen::Matrix<double, Eigen::Dynamic, 1> *DT; // Delta Theta Vector

	Eigen::Vector3f endEffector_current; 
	//Eigen::Vector3f end_effector_target; 

	Anim_State *anim; // Access to anim_state
};

#endif