#pragma once

// The link tree's findings, shared by buildTree (Kinematics.cpp) and validate (Validation.cpp).
// Private to viewer/builder/src.

#include "lagrange/viewer/builder/Document.hpp"

#include <string>
#include <vector>

namespace lagrange::viewer::builder::detail {

struct TreeFinding {
    std::string item;     ///< the link or joint it is about
    std::string message;  ///< one sentence
};

struct TreeAnalysis {
    std::vector<int> parentJoint;       ///< per link: the first joint whose child it is, or -1
    std::vector<int> roots;             ///< links no joint has as its child, in document order
    std::vector<TreeFinding> findings;  ///< empty when the joints make one tree
};

/// Every joint's parent and child exist and differ, no link has two parents, one root, no
/// loops; each problem once, in document order.
[[nodiscard]] TreeAnalysis analyseTree(const Document& document);

}  // namespace lagrange::viewer::builder::detail
