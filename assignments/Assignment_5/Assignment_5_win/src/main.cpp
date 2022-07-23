////////////////////////////////////////////////////////////////////////
//
//
//  Assignment 5 of SUTD Course 50.017 (May-Aug Term, 2022)
//
//     Linkage Design
//
//  2022-July-21
//
//
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include "eigen-3.4.0/Eigen/Eigen"

#include "shaderSource.h"
#include "shader.h"
#include "Linkages.h"

using namespace std;
using namespace Eigen;


#define MAX_BUFFER_SIZE            1024

#define _ROTATE_FACTOR              0.005f
#define _SCALE_FACTOR               0.01f
#define _TRANS_FACTOR               0.02f

#define _Z_NEAR                     0.001f
#define _Z_FAR                      100.0f



/***********************************************************************/
/**************************   global variables   ***********************/
/***********************************************************************/


// Window size
unsigned int winWidth  = 1200;
unsigned int winHeight = 800;

// Camera
glm::vec3 camera_position = glm::vec3 (0.0f, 0.0f, 12.5f);
glm::vec3 camera_target = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 camera_up = glm::vec3(0.0f, 1.0f, 0.0f);
float camera_fovy = 45.0f;    
glm::mat4 projection;

// Mouse interaction 
bool leftMouseButtonHold = false;
bool isFirstMouse = true;
float prevMouseX;
float prevMouseY;
vector<glm::mat4> modelMatrixVec;
vector<glm::mat4> localMatrixVec;
vector<glm::mat4> worldMatrixVec;

// Animation Mod
bool isAnimation = false;
static int animationInterval = 1;
double preTimer = 0.0;
int    curFrame = 1;
bool   curveShow = true;

 // Mesh color table
glm::vec3 colorTable[13] =
 {
    glm::vec3(0.8, 0.9, 0.8),
    glm::vec3(0.75, 0.85, 0.95),
    glm::vec3(0.75, 0.95, 0.85),
    glm::vec3(0.85, 0.95, 0.75),
    glm::vec3(0.85, 0.75, 0.95),
    glm::vec3(0.95, 0.75, 0.85),
    glm::vec3(0.95, 0.85, 0.75),
    glm::vec3(0.9, 0.8, 0.8),
    glm::vec3(0.8, 0.8, 0.9),
    glm::vec3(0.9, 0.9, 0.8),
    glm::vec3(0.9, 0.8, 0.9),

    glm::vec3(0.99, 0.4, 0.7),
    glm::vec3(0.85, 0.85, 0.85)
 };


//linkages mode
int caseLinks = 1;
const int MaxLinkPartsNum = 30;
double motionSpeed = 1.0;

// declaration
void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);



///=========================================================================================///
///                             Functions to Transform the Scene  
///=========================================================================================///

void InitModelMatrix(int partsNum)
{
    modelMatrixVec.clear();
    localMatrixVec.clear();
    worldMatrixVec.clear();
    for(int i=0;i<partsNum;i++)
    {
        modelMatrixVec.push_back(glm::mat4(1.0f));
        localMatrixVec.push_back(glm::mat4(1.0f));
        worldMatrixVec.push_back(glm::mat4(1.0f));
    }
}

void RotateModel(float angle, glm::vec3 axis)
{
    for(auto &modelMatrix : modelMatrixVec)
    {
        glm::vec3 rotateCenter = glm::vec3(modelMatrix[3][0], modelMatrix[3][1], modelMatrix[3][2]);

        glm::mat4 rotateMatrix = glm::mat4(1.0f);
        rotateMatrix = glm::translate(rotateMatrix, rotateCenter);
        rotateMatrix = glm::rotate(rotateMatrix, angle, axis);
        rotateMatrix = glm::translate(rotateMatrix, -rotateCenter);

        modelMatrix = rotateMatrix * modelMatrix;
    }
}

void TranslateModel(glm::vec3 transVec)
{
    for(auto &modelMatrix : modelMatrixVec)
    {
        glm::mat4 translateMatrix = glm::mat4(1.0f);
        translateMatrix = glm::translate(translateMatrix, transVec);

        modelMatrix = translateMatrix * modelMatrix;
    }
}

void ScaleModel(float scale)
{
    for(auto &modelMatrix : modelMatrixVec)
    {
        glm::vec3 scaleCenter = glm::vec3(modelMatrix[3][0], modelMatrix[3][1], modelMatrix[3][2]);

        glm::mat4 scaleMatrix = glm::mat4(1.0f);
        scaleMatrix = glm::translate(scaleMatrix, scaleCenter);
        scaleMatrix = glm::scale(scaleMatrix, glm::vec3(scale, scale, scale));
        scaleMatrix = glm::translate(scaleMatrix, -scaleCenter);

        modelMatrix = scaleMatrix * modelMatrix;
    }
}




///=========================================================================================///
///                                    Callback Functions
///=========================================================================================///

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}


// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.

    glViewport(0, 0, width, height);

    winWidth  = width;
    winHeight = height;
}


// glfw: whenever a key is pressed, this callback is called
// press 'SPACE" for animation
// press '0'-'6' for diff types of linkages
// ----------------------------------------------------------------------
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        isAnimation = !isAnimation;
    }

    if (key == GLFW_KEY_1 && action == GLFW_PRESS)
    {
        isAnimation = false;
        caseLinks = 1;
        motionSpeed = 1.0;
    }

    if (key == GLFW_KEY_2 && action == GLFW_PRESS)
    {
        isAnimation = false;
        caseLinks = 2;
        motionSpeed = 1.0;
    }
    if (key == GLFW_KEY_3 && action == GLFW_PRESS)
    {
        isAnimation = false;
        caseLinks = 3;
        motionSpeed = 1.0;
    }
    if (key == GLFW_KEY_4 && action == GLFW_PRESS)
    {
        isAnimation = false;
        caseLinks = 4;
        motionSpeed = 1.0;
    }

    if (key == GLFW_KEY_D && action == GLFW_PRESS)
    {
        if(abs(motionSpeed) < 20)
        {
            motionSpeed = motionSpeed * 1.1;
            printf("motionSpeed; %.3f \n", motionSpeed);
        }
    }
    if (key == GLFW_KEY_A && action == GLFW_PRESS)
    {
        if(abs(motionSpeed) > 0.02)
        {
            motionSpeed = motionSpeed / 1.1;
            printf("motionSpeed; %.3f \n", motionSpeed);
        }
    }

    if (key == GLFW_KEY_C && action == GLFW_PRESS)
        curveShow = !curveShow;
}


void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        leftMouseButtonHold = true;
    }
    else
    {
    	leftMouseButtonHold = false;
    }
}


// glfw: whenever the cursor moves, this callback is called
// -------------------------------------------------------
void cursor_pos_callback(GLFWwindow* window, double mouseX, double mouseY)
{
	float  dx, dy;
	float  nx, ny, scale, angle;
    

	if ( leftMouseButtonHold )
	{
		if (isFirstMouse)
	    {
	        prevMouseX = mouseX;
	        prevMouseY = mouseY;
	        isFirstMouse = false;
	    }

	    else
	    {
            if( glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS )
            {
                float dx =         _TRANS_FACTOR * (mouseX - prevMouseX);
                float dy = -1.0f * _TRANS_FACTOR * (mouseY - prevMouseY); // reversed since y-coordinates go from bottom to top

                prevMouseX = mouseX;
                prevMouseY = mouseY;

                TranslateModel( glm::vec3(dx, dy, 0) );  
            }

            else
            {
                float dx =   mouseX - prevMouseX;
                float dy = -(mouseY - prevMouseY); // reversed since y-coordinates go from bottom to top

                prevMouseX = mouseX;
                prevMouseY = mouseY;

               // Rotation
                nx    = -dy;
                ny    =  dx;
                scale = sqrt(nx*nx + ny*ny);

                // We use "ArcBall Rotation" to compute the rotation axis and angle based on the mouse motion
                nx    = nx / scale;
                ny    = ny / scale;
                angle = scale * _ROTATE_FACTOR;

                RotateModel( angle, glm::vec3(nx, ny, 0.0f) );
            }
	    }  
	}  

	else
	{
		isFirstMouse = true;
	}
}


// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xOffset, double yOffset)
{
	float scale = 1.0f + _SCALE_FACTOR * yOffset;

	ScaleModel( scale ); 
}




///=========================================================================================///
///                                      Main Function
///=========================================================================================///

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(winWidth, winHeight, "Assignment 5 - Linkage Design", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSwapInterval(animationInterval);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    // build and compile our shader program
    // ------------------------------------
    // vertex shader
    shader myShader;
    myShader.setUpShader(vertexShaderSource,fragmentShaderSource);

    //linkages initialization
    auto _links = new Linkages();

    // Load input mesh data
    vector<vector<float>> verList;          // This is the list of vertices and normals for rendering
    vector<vector<unsigned>> triList;       // This is the list of faces for rendering

    ReInitial :
    _links->motionSpeed = motionSpeed;
    _links->InitLinkages(caseLinks, verList, triList);

    int sceneObjNum = _links->linksNum + _links->rJointNum + 1;

    InitModelMatrix(sceneObjNum);
    _links->UpdateLocalMatrix(localMatrixVec);

    // create buffers/arrays
    unsigned int VBO[MaxLinkPartsNum], VAO[MaxLinkPartsNum], EBO[MaxLinkPartsNum];
    glGenVertexArrays(sceneObjNum, VAO);
    glGenBuffers(sceneObjNum, VBO);
    glGenBuffers(sceneObjNum, EBO);

    for(int i=0;i<sceneObjNum;i++)
    {
        glBindVertexArray(VAO[i]);

        // load data into vertex buffers
        glBindBuffer(GL_ARRAY_BUFFER, VBO[i]);
        glBufferData(GL_ARRAY_BUFFER, verList[i].size() * sizeof(float), &verList[i][0], GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[i]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, triList[i].size() * sizeof(unsigned int), &triList[i][0], GL_STATIC_DRAW);

        // set the vertex attribute pointers
        // vertex Positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *) 0);
        glEnableVertexAttribArray(0);

        // vertex normals
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), ((void *) (3 * sizeof(float))));
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);
    }
 
    // as we only have a single shader, we could also just activate our shader once beforehand if we want to 
    myShader.use();


    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.95f, 0.95f, 0.95f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // view/projection transformations
        projection = glm::perspective(glm::radians(camera_fovy), (float)winWidth / (float)winHeight, _Z_NEAR, _Z_FAR);
        glm::mat4 view = glm::lookAt(camera_position, camera_target, camera_up);

        glUniformMatrix4fv(glGetUniformLocation(myShader.ID, "projection"), 1, GL_FALSE, &projection[0][0]);
        glUniformMatrix4fv(glGetUniformLocation(myShader.ID, "view"), 1, GL_FALSE, &view[0][0]);
        glUniform3fv(glGetUniformLocation(myShader.ID, "viewPos"), 1, &camera_position[0]);

        // switch diff types of linkages
        if(caseLinks != _links->linksCase)
        {
            glDeleteVertexArrays(sceneObjNum, VAO);
            glDeleteBuffers(sceneObjNum, VBO);
            curFrame = 1;
            goto ReInitial;
        }
        // update linkages local Matrix in animation
        if(isAnimation)
        {
            if(abs(_links->motionSpeed) != motionSpeed)
            {
                double dire = _links->motionSpeed/abs(_links->motionSpeed);
                _links->motionSpeed = motionSpeed*dire;
            }
            _links->InputDriverAngle(curFrame);
            _links->SolveLinkages();
            _links->UpdateLocalMatrix(localMatrixVec);
            curFrame++;
        }

        //render each link
        for(int i=0;i<_links->linksNum + _links->rJointNum;i++)
        {
            worldMatrixVec[i] = modelMatrixVec[i]*localMatrixVec[i];
            glUniformMatrix4fv(glGetUniformLocation(myShader.ID, "model"), 1, GL_FALSE, &worldMatrixVec[i][0][0]);

            if(i == _links->groundLinkID || i >= _links->linksNum)
                glUniform3fv(glGetUniformLocation(myShader.ID, "meshColor"), 1, &colorTable[12][0]);
            else
                glUniform3fv(glGetUniformLocation(myShader.ID, "meshColor"), 1, &colorTable[i%11][0]);

            // render the triangle
            glBindVertexArray(VAO[i]);
            glDrawElements(GL_TRIANGLES, triList[i].size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
        if(curveShow)
        {
            int i = _links->linksNum + _links->rJointNum;
            worldMatrixVec[i] = modelMatrixVec[i]*localMatrixVec[i];
            glUniformMatrix4fv(glGetUniformLocation(myShader.ID, "model"), 1, GL_FALSE, &worldMatrixVec[i][0][0]);
            glUniform3fv(glGetUniformLocation(myShader.ID, "meshColor"), 1, &colorTable[11][0]);
            glBindVertexArray(VAO[i]);
            glDrawElements(GL_TRIANGLES, triList[i].size(), GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(sceneObjNum, VAO);
    glDeleteBuffers(sceneObjNum, VBO);
    glDeleteProgram(myShader.ID);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

