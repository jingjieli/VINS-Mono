#include "visualizer.h"
#include "global_parameters.h"

Visualizer::Visualizer()
{

}
    
Visualizer::~Visualizer()
{

}

void Visualizer::init()
{
    is_running = true;
    rendering_thread = std::thread(&Visualizer::start, this);
}

void Visualizer::quit()
{
    is_running = false;
    rendering_thread.join();
}
    
void Visualizer::start()
{
    pangolin::CreateWindowAndBind("Visualization", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need 
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowKeyframe("menu.Show Keyframe", true, true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
    pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);
    pangolin::Var<bool> menuShowEdge("menu.Show Edge", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    Twc.SetIdentity();

    bool is_followed = true;

    while(!pangolin::ShouldQuit() && is_running)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

        // make a local copy
        pangolin::OpenGlMatrix twc;
        render_mtx.lock();
        twc = Twc;
        render_mtx.unlock();

        std::cout << "twc: " << twc << std::endl;

        if (menuFollowCamera && is_followed)
        {
            s_cam.Follow(twc);
        }
        else if (menuFollowCamera && !is_followed)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(twc);
            is_followed = true;
        }
        else if (!menuFollowCamera && is_followed)
        {
            is_followed = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        drawCurrentCamera(twc);

        if (menuShowKeyframe)
        {
            drawKeyframes();
            drawPoseGraph();
        }

        if (menuShowPoints)
        {
            drawPointCloud();
        }

        if (menuShowPath)
        {
            drawTrajectory();
            drawNoLoopPath();
            drawPoseGraphPath();
            //drawRelocPath();
        }

        if (menuShowEdge)
        {
            drawLoopEdge();
        }

        pangolin::FinishFrame();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    printf("[Visualizer] visualization thread stops\n");
}

void Visualizer::updateCameraPose(const POSE_MSG &pose_msg)
{
    
}

void Visualizer::updateCameraPose(const Eigen::Vector3d &P, const Eigen::Matrix3d &R)
{   
    // make a local copy
    pangolin::OpenGlMatrix twc;

    poseToGlMatrix(P, R, twc);

    render_mtx.lock();
    Twc = twc;
    render_mtx.unlock();
}

void Visualizer::addKeyframePose(const POSE_MSG &pose_msg)
{
    Eigen::Vector3d P = Eigen::Vector3d(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
    Eigen::Matrix3d R = Eigen::Quaterniond(
            pose_msg.orientation_z,
            pose_msg.orientation_x,
            pose_msg.orientation_y,
            pose_msg.orientation_z
        ).toRotationMatrix();

    pangolin::OpenGlMatrix M;
    poseToGlMatrix(P, R, M);
    
    render_mtx.lock();
    keyframe_poses.push_back(M);
    render_mtx.unlock();
}

void Visualizer::updatePointCloud(const std::vector<Eigen::Vector3d> &points)
{
    render_mtx.lock();
    point_cloud = points;
    render_mtx.unlock();
}

void Visualizer::updateTrajectory(const POSE_MSG &pose_msg)
{
    Eigen::Vector3d P(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
    
    render_mtx.lock();
    trajectory.push_back(P);
    render_mtx.unlock();
}

void Visualizer::updateNoLoopPath(const POSE_MSG &pose_msg)
{
    Eigen::Vector3d P(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
    
    render_mtx.lock();
    no_loop_path.push_back(P);
    render_mtx.unlock();
}

void Visualizer::updateRelocPath(const POSE_MSG &pose_msg)
{
    Eigen::Vector3d P(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
    
    render_mtx.lock();
    reloc_path.push_back(P);
    render_mtx.unlock();
}

void Visualizer::addLoopEdge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1)
{
    EDGE edge;
    edge.p0 = p0;
    edge.p1 = p1;

    render_mtx.lock();
    loop_edges.push_back(edge);
    render_mtx.unlock();
}

void Visualizer::updatePoseGraph(const std::vector<POSE_MSG> &path)
{
    for (const auto &pose_msg : path)
    {
        Eigen::Vector3d P = Eigen::Vector3d(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);
        Eigen::Matrix3d R = Eigen::Quaterniond(
            pose_msg.orientation_z,
            pose_msg.orientation_x,
            pose_msg.orientation_y,
            pose_msg.orientation_z
        ).toRotationMatrix();

        pangolin::OpenGlMatrix M;
        poseToGlMatrix(P, R, M);

        render_mtx.lock();
        pose_graph_poses.push_back(M);
        render_mtx.unlock();
    }
}

void Visualizer::updatePoseGraphPath(const std::vector<POSE_MSG> &path)
{
    render_mtx.lock();
    pose_graph_traj.clear();
    for (const auto &pose_msg : path)
    {
        Eigen::Vector3d P = Eigen::Vector3d(pose_msg.position_x, pose_msg.position_y, pose_msg.position_z);

        pose_graph_traj.push_back(P);
    }
    render_mtx.unlock();
}

void Visualizer::resetPoseGraph()
{
    render_mtx.lock();
    pose_graph_poses.clear();
    pose_graph_traj.clear();
    render_mtx.unlock();
}

void Visualizer::drawCurrentCamera(const pangolin::OpenGlMatrix &M)
{
    const float &w = 0.08f;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

    glMultMatrixd(M.m);

    glLineWidth(3);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);
    
    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);
    
    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);

    glEnd();
    glPopMatrix();
}

void Visualizer::drawKeyframes()
{
    if (keyframe_poses.size() == 0)
        return;

    const float &w = 0.05f;
    const float h = w * 0.75;
    const float z = w * 0.6;

    render_mtx.lock();
    for (unsigned int i = 0; i < keyframe_poses.size(); ++i)
    {
        pangolin::OpenGlMatrix M;

        M = keyframe_poses[i];

        glPushMatrix();

        glMultMatrixd(M.m);

        glLineWidth(1);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }      
    render_mtx.unlock();  
}

void Visualizer::drawPointCloud()
{
    if (point_cloud.size() == 0)
        return;

    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0);

    render_mtx.lock();
    for (auto point : point_cloud)
    {
        glVertex3f(point.x(), point.y(), point.z());
    }
    render_mtx.unlock();

    glEnd();
}

void Visualizer::drawTrajectory()
{
    if (trajectory.size() == 0)
        return;

    glLineWidth(2);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);

    render_mtx.lock();
    for (unsigned int i = 0; i < trajectory.size() - 1; ++i)
    {
        glVertex3f(
            (float) trajectory[i].x(),
            (float) trajectory[i].y(),
            (float) trajectory[i].z()
        );

        glVertex3f(
            (float) trajectory[i + 1].x(),
            (float) trajectory[i + 1].y(),
            (float) trajectory[i + 1].z()
        );
    }
    render_mtx.unlock();

    glEnd();
}

void Visualizer::drawNoLoopPath()
{
    if (no_loop_path.size() == 0)
        return;

    glLineWidth(2);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);

    render_mtx.lock();
    for (unsigned int i = 0; i < no_loop_path.size() - 1; ++i)
    {
        glVertex3f(
            (float) no_loop_path[i].x(),
            (float) no_loop_path[i].y(),
            (float) no_loop_path[i].z()
        );

        glVertex3f(
            (float) no_loop_path[i + 1].x(),
            (float) no_loop_path[i + 1].y(),
            (float) no_loop_path[i + 1].z()
        );
    }
    render_mtx.unlock();

    glEnd();
}

void Visualizer::drawRelocPath()
{
    if (reloc_path.size() == 0)
        return;

    glLineWidth(2);
    glColor3f(0.6f, 0.2f, 0.2f);
    glBegin(GL_LINES);

    render_mtx.lock();
    for (unsigned int i = 0; i < reloc_path.size() - 1; ++i)
    {
        glVertex3f(
            (float) reloc_path[i].x(),
            (float) reloc_path[i].y(),
            (float) reloc_path[i].z()
        );

        glVertex3f(
            (float) reloc_path[i + 1].x(),
            (float) reloc_path[i + 1].y(),
            (float) reloc_path[i + 1].z()
        );
    }
    render_mtx.unlock();

    glEnd();
}

void Visualizer::drawLoopEdge()
{
    if (loop_edges.size() == 0)
        return;

    glLineWidth(2);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);

    render_mtx.lock();
    for (unsigned int i = 0; i < loop_edges.size(); ++i)
    {
        glVertex3f(
            (float) loop_edges[i].p0.x(),
            (float) loop_edges[i].p0.y(),
            (float) loop_edges[i].p0.z()
        );

        glVertex3f(
            (float) loop_edges[i].p1.x(),
            (float) loop_edges[i].p1.y(),
            (float) loop_edges[i].p1.z()
        );
    }
    render_mtx.unlock();

    glEnd();
}

void Visualizer::drawPoseGraph()
{
    if (pose_graph_poses.size() == 0)
        return;

    const float &w = 0.05f;
    const float h = w * 0.75;
    const float z = w * 0.6;

    render_mtx.lock();
    // draw all poses in graph
    for (unsigned int i = 0; i < pose_graph_poses.size(); ++i)
    {
        pangolin::OpenGlMatrix M;

        M = pose_graph_poses[i];

        glPushMatrix();

        glMultMatrixd(M.m);

        glLineWidth(1);
        glColor3f(1.0f, 0.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }  

    render_mtx.unlock();  
}

void Visualizer::drawPoseGraphPath()
{
    if (pose_graph_traj.size() == 0)
        return;

    // draw line between poses
    glLineWidth(2);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);

    render_mtx.lock();
    for (unsigned int i = 0; i < pose_graph_traj.size() - 1; ++i)
    {
        glVertex3f(
            (float) pose_graph_traj[i].x(),
            (float) pose_graph_traj[i].y(),
            (float) pose_graph_traj[i].z()
        );

        glVertex3f(
            (float) pose_graph_traj[i + 1].x(),
            (float) pose_graph_traj[i + 1].y(),
            (float) pose_graph_traj[i + 1].z()
        );
    }   
    render_mtx.unlock();

    glEnd();
}

void Visualizer::poseToGlMatrix(const Eigen::Vector3d &P, const Eigen::Matrix3d &R, pangolin::OpenGlMatrix &M)
{
    M.m[0] = R(0, 0);
    M.m[1] = R(1, 0);
    M.m[2] = R(2, 0);
    M.m[3] = 0.0;

    M.m[4] = R(0, 1);
    M.m[5] = R(1, 1);
    M.m[6] = R(2, 1);
    M.m[7] = 0.0;

    M.m[8] = R(0, 2);
    M.m[9] = R(1, 2);
    M.m[10] = R(2, 2);
    M.m[11] = 0.0;

    M.m[12] = P.x();
    M.m[13] = P.y();
    M.m[14] = P.z();
    M.m[15] = 1.0;
}