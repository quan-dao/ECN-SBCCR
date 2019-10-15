#include <ecn_sensorbased/pioneer_cam.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_common/vpQuadProg.h>
#include <ecn_common/visp_utils.h>

// using namespace std;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_node");
    PioneerCam robot; 

    // pose error gain
    const double lv = .5;
    // constraints gain
    const double lc = 2;
    geometry_msgs::Pose2D target;

    int it = 0;
    vpColVector q_d(4);  // [v, omega, qp_d, qt_d] : all joints velocity 

    // QP solver
    vpQuadProg qp;

    // Define Q 
    vpMatrix Q(2, 4);
    Q[0][0] = 1.;
    Q[1][1] = 1.;

    // Define Jw
    vpMatrix K_inv(2, 2);
    K_inv[0][0] = 1 / robot.radius();
    K_inv[0][1] = robot.base() / robot.radius();
    K_inv[1][0] = 1 / robot.radius();
    K_inv[1][1] = -robot.base() / robot.radius();

    vpMatrix Jw(2, 4);
    ecn::putAt(Jw, K_inv, 0, 0);

    // wheel upper limit
    vpColVector omega_max(4);
    for(int i = 0; i < 4; i++)
        omega_max[i] = robot.wmax(); 

    // Initialize feature point
    vpFeaturePoint s;

    const double alpha(1.5);
    const double beta(0.95);

    while(ros::ok())
    {
        it++;
        std::cout << "-------------" << std::endl;

        if(robot.ok())
        {
            // get robot and target positions to get position error
            target = robot.getTargetRelativePose();

            // Build interaction matrix
            vpColVector img_point(2);
            robot.getImagePoint(img_point);
            s.buildFrom(img_point[0], img_point[1], 1.);

            vpMatrix L = s.interaction();

            // Compute feature Jacobian
            vpMatrix Js = L * robot.getCamJacobian();

            // Compute boundary of s_dot
            vpColVector s_plus(2);
            s_plus = beta*robot.getCamLimits();
            vpColVector sd_up(2);
            sd_up = alpha * (s_plus - img_point);

            vpColVector sd_down(2);
            sd_down = -alpha * (-s_plus - img_point);

            // target linear & angular velocity of moving platform
            vpColVector r(2);
            r[0] = lv*(target.x - .1);  // v*
            r[1] = 5*lv*std::atan2(target.y, target.x);  // omega*
            
            // Define A
            vpMatrix A(1, 4);
            A[0][0] = r[1];
            A[0][1] = -r[0];

            // Define b
            vpColVector b(1);

            // Define C
            vpMatrix C(8, 4);
            ecn::putAt(C, Jw, 0, 0);
            ecn::putAt(C, -Jw, 2, 0);
            ecn::putAt(C, Js, 4, 0);
            ecn::putAt(C, -Js, 6, 0);

            // Define d
            vpColVector d(8);
            ecn::putAt(d, omega_max, 0);
            ecn::putAt(d, sd_up, 4);
            ecn::putAt(d, sd_down, 6);

            // Solve QP
            qp.solveQP(Q, r, A, b, C, d, q_d);


            std::cout << "q_d: " << q_d.t() << std::endl;

            robot.setVelocity(q_d);
        }
    }
}
