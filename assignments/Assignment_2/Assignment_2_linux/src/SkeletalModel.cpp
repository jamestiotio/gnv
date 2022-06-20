///=========================================================================================///
///
///                       Functions to be filled in for Assignment 2
///
///           IMPORTANT: you ONLY need to work on functions with TODO in this section
///
///=========================================================================================///

#include "SkeletalModel.h"
#include "Sphere.h"
#include "Cylinder.h"

///=========================================================================================///
///                                    Load .skel File
///=========================================================================================///

// TODO: Load the skeleton from file here, create hierarchy of joints
//       (i.e., set values for m_rootJoint and m_joints)
void SkeletalModel::loadSkeleton(const char *filename)
{
    string filenameStr(filename);
    if (filenameStr.find(".skel") == string::npos)
    {
        cout << "Error: Input file must be in .skel format." << endl;
        exit(1);
    }
    ifstream fin(filenameStr, ios::in);
    if (!fin)
    {
        cerr << "Unable to open the specified file. Please check again that everything is in proper order." << endl;
        exit(1);
    }

    cout << "Loading model skeleton file: " << filenameStr << endl;

    string lineBuffer;
    int numberOfJoints = 0;

    while (getline(fin, lineBuffer))
    {
        float x, y, z;
        int parentIdx;
        const char *skelJointData = lineBuffer.c_str();
        int match = sscanf(skelJointData, "%f %f %f %i", &x, &y, &z, &parentIdx);
        if (match != 4)
        {
            cout << "Failed to parse the SKEL file." << endl;
            exit(1);
        }

        // Create a new Joint instance
        Joint *joint = new Joint;

        // Set the joint's transformation matrix relative to its parent joint
        glm::mat4 jointTransformMatrix = glm::mat4(1.0f);
        jointTransformMatrix[3] = glm::vec4(x, y, z, 1);
        joint->transform = jointTransformMatrix;

        // Set the root joint if the parentIdx == -1
        if (parentIdx == -1)
        {
            // Set the root joint for skeletal model
            m_rootJoint = joint;
        }
        else if (parentIdx >= 0)
        {
            // Push the current joint instance to its parent children vector
            m_joints.at(parentIdx)->children.push_back(joint);
        }
        else
        {
            cerr << "Invalid parent joint index specified in the SKEL file! Aborting..." << endl;
            exit(1);
        }

        // Push the current joint instance to the skeletal model joints vector
        m_joints.push_back(joint);
        numberOfJoints++;
    }

    // Close the file
    fin.close();
    cout << "Done loading the SKEL file. Number of joints: " << numberOfJoints << endl;

    return;
}

///=========================================================================================///
///                         Compute transformations for Joints and Bones
///=========================================================================================///

void SkeletalModel::computeTransforms()
{
    if (m_joints.size() == 0)
        return;

    computeJointTransforms();

    computeBoneTransforms();
}

// Compute a transformation matrix for each joint (i.e., ball) of the skeleton
void SkeletalModel::computeJointTransforms()
{
    jointMatList.clear();

    m_matrixStack.clear();

    computeJointTransforms(m_rootJoint, m_matrixStack);
}

// TODO: You will need to implement this recursive helper function to traverse the joint hierarchy for computing transformations of the joints
void SkeletalModel::computeJointTransforms(Joint *joint, MatrixStack matrixStack)
{
    // Add the current joint's transformation matrix to the matrix stack
    matrixStack.push(glm::transpose(joint->transform));

    // Recursive call for each child joint
    for (Joint *j : joint->children)
    {
        computeJointTransforms(j, matrixStack);
    }

    // Push the top of the matrix stack to jointMatList
    jointMatList.push_back(matrixStack.top());

    // Pop the current joint's transformation matrix from the matrix stack
    matrixStack.pop();
}

// Compute a transformation matrix for each bone (i.e., cylinder) between each pair of joints in the skeleton
void SkeletalModel::computeBoneTransforms()
{
    boneMatList.clear();

    m_matrixStack.clear();

    computeBoneTransforms(m_rootJoint, m_matrixStack);
}

// TODO: You will need to implement this recursive helper function to traverse the joint hierarchy for computing transformations of the bones
void SkeletalModel::computeBoneTransforms(Joint *joint, MatrixStack matrixStack)
{
    // Add the current joint's transformation matrix to the matrix stack
    matrixStack.push(glm::transpose(joint->transform));

    // Translate in the Z direction
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(0, 0, 0.5f));

    for (Joint *j : joint->children)
    {
        glm::vec3 childVec = glm::vec3(j->transform[3]);

        glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), glm::vec3(0.01f, 0.01f, glm::length(childVec)));

        glm::vec3 z = glm::normalize(childVec);
        glm::vec3 y = glm::normalize(glm::cross(z, glm::vec3(0, 0, 1)));
        glm::vec3 x = glm::normalize(glm::cross(y, z));
        glm::mat3 rotation(x, y, z);
        glm::mat4 alignMatrix = glm::mat4(rotation);
        glm::mat4 drawBoxTransMatrix = alignMatrix * scaleMatrix * translationMatrix;

        // Push the drawBoxTransMatrix into the matrix stack
        matrixStack.push(glm::transpose(drawBoxTransMatrix));

        // Push the top of the matrix stack to boneMatList
        boneMatList.push_back(matrixStack.top());

        // Pop the drawBoxTransMatrix
        matrixStack.pop();

        // Recursive call for each child joint
        computeBoneTransforms(j, matrixStack);
    }

    // Pop the current joint node transform matrix
    matrixStack.pop();
}

///=========================================================================================///
///                              Set Joint Angles for Transform
///=========================================================================================///

// TODO: Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
void SkeletalModel::setJointTransform(int jointIndex, float angleX, float angleY, float angleZ)
{
    Joint *joint = m_joints[jointIndex];
    glm::mat4 rotationTransform = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), glm::vec3(0.0f, 0.0f, 1.0f));
    rotationTransform = glm::rotate(rotationTransform, glm::radians(angleY), glm::vec3(0.0f, 1.0f, 0.0f));
    rotationTransform = glm::rotate(rotationTransform, glm::radians(angleX), glm::vec3(1.0f, 0.0f, 0.0f));
    joint->transform[0] = rotationTransform[0];
    joint->transform[1] = rotationTransform[1];
    joint->transform[2] = rotationTransform[2];
}
