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
	IK_Solver(Anim_State *Anim, std::vector<Joint*> &joints, Effector *target);
	~IK_Solver() = default;

	// Build Jacobian
	//void build_jacobian();

	// Eval Jacobian

	// Inverse / Pseudo-Inverse Jacobian

	// Static Member Functions 
	// GLM to Eigen
	static Eigen::Vector3f glmToeig(const glm::vec3 &vec);
	static Eigen::Vector4f glmToeig(const glm::vec4 &vec);

public:
	Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> *jacobian; 

	Anim_State *anim;
};

#endif