////////////////////////////////////////////////////////////////////////
//
//
//  Assignment 1 of SUTD Course 50.017 (May-Aug Term, 2022)
//
//    Mesh Viewer
//
//  2022-May-26
//
//
////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <sstream>
#include <vector>
#include <set>
#include <fstream>
#include <bits/stdc++.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "shaderSource.h"
#include "shader.h"

using namespace std;

#define MAX_BUFFER_SIZE 1024

#define _ROTATE_FACTOR 0.005f
#define _SCALE_FACTOR 0.01f
#define _TRANS_FACTOR 0.02f

#define _Z_NEAR 0.001f
#define _Z_FAR 100.0f

/***********************************************************************/
/**************************   global variables   ***********************/
/***********************************************************************/

// Window size
unsigned int winWidth = 800;
unsigned int winHeight = 600;

// Camera
glm::vec3 camera_position = glm::vec3(0.0f, 0.0f, 15.0f);
glm::vec3 camera_target = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 camera_up = glm::vec3(0.0f, 1.0f, 0.0f);
float camera_fovy = 45.0f;
glm::mat4 projection;

// Mouse interaction
bool leftMouseButtonHold = false;
bool isFirstMouse = true;
float prevMouseX;
float prevMouseY;
glm::mat4 modelMatrix = glm::mat4(1.0f);

// Mesh color table
glm::vec3 colorTable[4] = {
    glm::vec3(0.6, 1.0, 0.6),
    glm::vec3(1.0, 0.6, 0.6),
    glm::vec3(0.6, 0.6, 1.0),
    glm::vec3(1.0, 1.0, 0.6)};

// Mesh rendering color
int colorID = 0;
glm::vec3 meshColor;

// declaration
void processInput(GLFWwindow *window);
void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

/******************************************************************************/
/***************   Functions to be filled in for Assignment 1    **************/
/***************    IMPORTANT: you ONLY need to work on these    **************/
/***************                functions in this section        **************/
/******************************************************************************/

// TODO: insert your code in this function for Mesh Loading
//       1) store vertices and normals in verList with order (v.x, v.y, v.z, n.x, n.y, n.z)
//       2) store vertex indices of each triangle in triList
int LoadInput(vector<float> &verList, vector<unsigned> &triList, string filename)
{
    if (filename.find(".obj") == string::npos)
    {
        cout << "Error: Input file must be in .obj format." << endl;
        return 1;
    }

    ifstream fin(filename, ios::in);
    if (!fin)
    {
        return 1;
    }

    cout << "Loading OBJ file: " << filename << endl;

    string lineBuffer;

    vector<vector<float>> vertices;
    vector<vector<float>> normals;
    set<pair<unsigned, unsigned>> vertex_normal_pairs;
    unsigned numVertices = 0;
    unsigned numNormals = 0;
    unsigned numFaces = 0;

    while (getline(fin, lineBuffer))
    {
        if (lineBuffer.substr(0, 2) == "v ")
        {
            istringstream v(lineBuffer.substr(2));
            float x, y, z;
            v >> x >> y >> z;
            vector<float> vertex = {x, y, z};
            vertices.push_back(vertex);
            numVertices++;
        }
        else if (lineBuffer.substr(0, 2) == "vn")
        {
            istringstream vn(lineBuffer.substr(2));
            float x, y, z;
            vn >> x >> y >> z;
            // Normalize the normal vectors
            float length = sqrt(x * x + y * y + z * z);
            x /= length;
            y /= length;
            z /= length;
            vector<float> normal = {x, y, z};
            normals.push_back(normal);
            numNormals++;
        }
        else if (lineBuffer.substr(0, 2) == "f ")
        {
            unsigned int v1, v2, v3;
            unsigned int t1, t2, t3;
            unsigned int n1, n2, n3;
            const char *face = lineBuffer.c_str();
            int match = sscanf(face, "f %i/%i/%i %i/%i/%i %i/%i/%i", &v1, &t1, &n1,
                               &v2, &t2, &n2, &v3, &t3, &n3);
            if (match != 9)
                cout << "Invalid face element line detected in the OBJ file. Aborting..." << endl;
            if (v1 <= 0 || v2 <= 0 || v3 <= 0 || n1 <= 0 || n2 <= 0 || n3 <= 0)
            {
                cout << "Invalid vertex or normal index found in the OBJ file. Aborting..." << endl;
                return 1;
            }
            triList.push_back(v1 - 1);
            triList.push_back(v2 - 1);
            triList.push_back(v3 - 1);

            pair<unsigned, unsigned> vertex_normal_pair1 = make_pair(v1 - 1, n1 - 1);
            pair<unsigned, unsigned> vertex_normal_pair2 = make_pair(v2 - 1, n2 - 1);
            pair<unsigned, unsigned> vertex_normal_pair3 = make_pair(v3 - 1, n3 - 1);

            // Add the vertex-normal pair only if it is not in the list yet
            if (vertex_normal_pairs.find(vertex_normal_pair1) == vertex_normal_pairs.end())
            {
                vertex_normal_pairs.insert(vertex_normal_pair1);
            }

            if (vertex_normal_pairs.find(vertex_normal_pair2) == vertex_normal_pairs.end())
            {
                vertex_normal_pairs.insert(vertex_normal_pair2);
            }

            if (vertex_normal_pairs.find(vertex_normal_pair3) == vertex_normal_pairs.end())
            {
                vertex_normal_pairs.insert(vertex_normal_pair3);
            }

            numFaces++;
        }
    }

    // Check that there are no duplicate vertex indices in the vertex-normal pairs
    if (vertex_normal_pairs.size() > numVertices)
    {
        cout << "Contradicting vertex-normal pair information found in the OBJ file! Aborting..." << endl;
        return 1;
    }
    if (vertex_normal_pairs.size() < numVertices)
    {
        cout << "Note that the OBJ file contains some vertices without corresponding normal vectors that are not used." << endl;
    }

    // Print number of vertices, normals, and faces
    cout << "Number of vertices: " << numVertices << endl;
    cout << "Number of normals: " << numNormals << endl;
    cout << "Number of faces: " << numFaces << endl;

    // Sort the vertex-normal pairs in ascending order based on the vertex index (first element of the pair)
    vector<pair<unsigned, unsigned>> vertex_normal_pairs_sorted(vertex_normal_pairs.begin(), vertex_normal_pairs.end());
    sort(vertex_normal_pairs_sorted.begin(), vertex_normal_pairs_sorted.end(),
         [](const pair<unsigned, unsigned> &a, const pair<unsigned, unsigned> &b)
         {
             return a.first < b.first;
         });

    // Add all vertex-normal pairs based on their indices and push to verList
    for (auto vertex_normal_pair : vertex_normal_pairs_sorted)
    {
        unsigned vertex_index = vertex_normal_pair.first;
        unsigned normal_index = vertex_normal_pair.second;
        float x = vertices[vertex_index][0];
        float y = vertices[vertex_index][1];
        float z = vertices[vertex_index][2];
        float nx = normals[normal_index][0];
        float ny = normals[normal_index][1];
        float nz = normals[normal_index][2];
        verList.push_back(x);
        verList.push_back(y);
        verList.push_back(z);
        verList.push_back(nx);
        verList.push_back(ny);
        verList.push_back(nz);
    }

    // Close the file
    fin.close();
    cout << "Done loading the OBJ file." << endl;
    return 0;
}

// TODO: insert your code in this function for Mesh Coloring
void SetMeshColor(int &colorID)
{
    // Set the meshColor using the colorTable and current colorID
    for (int i = 0; i < sizeof(meshColor) / sizeof(meshColor[0]); i++)
    {
        meshColor[i] = colorTable[colorID][i];
    }

    // Update colorID
    colorID = (colorID + 1) % 4;
}

// TODO: insert your code in this function for Mesh Transformation (Rotation)
void RotateModel(float angle, glm::vec3 axis)
{
    // Rotate the modelMatrix by angle around axis
    modelMatrix = glm::rotate(modelMatrix, angle, axis);
}

// TODO: insert your code in this function for Mesh Transformation (Translation)
void TranslateModel(glm::vec3 transVec)
{
    // Translate the modelMatrix by transVec
    modelMatrix = glm::translate(modelMatrix, transVec);
}

// TODO: insert your code in this function for Mesh Transformation (Scaling)
void ScaleModel(float scale)
{
    // Scale the modelMatrix by scale
    modelMatrix = glm::scale(modelMatrix, glm::vec3(scale, scale, scale));
}

/******************************************************************************/
/***************                  Callback Function              **************/
/******************************************************************************/

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.

    glViewport(0, 0, width, height);

    winWidth = width;
    winHeight = height;
}

// glfw: whenever a key is pressed, this callback is called
// ----------------------------------------------------------------------
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_C && action == GLFW_PRESS)
    {
        SetMeshColor(colorID);
    }
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
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
void cursor_pos_callback(GLFWwindow *window, double mouseX, double mouseY)
{
    float dx, dy;
    float nx, ny, scale, angle;

    if (leftMouseButtonHold)
    {
        if (isFirstMouse)
        {
            prevMouseX = mouseX;
            prevMouseY = mouseY;
            isFirstMouse = false;
        }

        else
        {
            if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            {
                float dx = _TRANS_FACTOR * (mouseX - prevMouseX);
                float dy = -1.0f * _TRANS_FACTOR * (mouseY - prevMouseY); // reversed since y-coordinates go from bottom to top

                prevMouseX = mouseX;
                prevMouseY = mouseY;

                TranslateModel(glm::vec3(dx, dy, 0));
            }

            else
            {
                float dx = mouseX - prevMouseX;
                float dy = -(mouseY - prevMouseY); // reversed since y-coordinates go from bottom to top

                prevMouseX = mouseX;
                prevMouseY = mouseY;

                // Rotation
                nx = -dy;
                ny = dx;
                scale = sqrt(nx * nx + ny * ny);

                // We use "ArcBall Rotation" to compute the rotation axis and angle based on the mouse motion
                nx = nx / scale;
                ny = ny / scale;
                angle = scale * _ROTATE_FACTOR;

                RotateModel(angle, glm::vec3(nx, ny, 0.0f));
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
void scroll_callback(GLFWwindow *window, double xOffset, double yOffset)
{
    float scale = 1.0f + _SCALE_FACTOR * yOffset;

    ScaleModel(scale);
}

/******************************************************************************/
/***************                    Main Function                **************/
/******************************************************************************/

int main(int argc, char **argv)
{
    string filename;
    if (argc > 1)
    {
        filename = argv[1];
    }
    else
    {
        cout << "Please input a filename!" << endl;
        return 1;
    }

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
    GLFWwindow *window = glfwCreateWindow(winWidth, winHeight, "[James Raphael Tiovalen] Assignment 1 - Mesh Viewer", NULL, NULL);
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
    myShader.setUpShader(vertexShaderSource, fragmentShaderSource);

    // Load input mesh data
    vector<float> verList;    // This is the list of vertices and normals for rendering
    vector<unsigned> triList; // This is the list of faces for rendering
    int status = LoadInput(verList, triList, filename);

    if (status == 1)
    {
        cout << "Unable to load the specified file. Please check again that everything is in proper order." << endl;
        return 1;
    }

    // create buffers/arrays
    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);

    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, verList.size() * sizeof(float), &verList[0], GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, triList.size() * sizeof(unsigned int), &triList[0], GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // vertex normals
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), ((void *)(3 * sizeof(float))));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);

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

        // render the loaded model
        // glm::vec3 aColor = glm::vec3 (0.6f, 1.0f, 0.6f);
        glUniformMatrix4fv(glGetUniformLocation(myShader.ID, "model"), 1, GL_FALSE, &modelMatrix[0][0]);
        glUniform3fv(glGetUniformLocation(myShader.ID, "meshColor"), 1, &colorTable[colorID][0]);
        glUniform3fv(glGetUniformLocation(myShader.ID, "viewPos"), 1, &camera_position[0]);

        // render the triangle
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, triList.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(myShader.ID);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}
