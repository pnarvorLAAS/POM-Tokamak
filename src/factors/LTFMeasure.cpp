#include <infuse_pom_tokamak/factors/LTFMeasure.hpp>


namespace tokamak
{
    namespace LTFMeasure
    {
        Vector RotationFactor::evaluateError(const Pose2& site_local, const Pose3& site_robot, boost::optional<Matrix&> Hi, boost::optional<Matrix&> Hj) const
        {
            // Transform Rot2 of site_local into Rot3
            OptionalJacobian<3,3>::Jacobian J_SL3D_i;
            const Rot3 rot_site_local = Rot3::Expmap(Vector3(0,0,site_local.theta()),J_SL3D_i);

            // Create true Jacobian
            OptionalJacobian<3,3>::Jacobian J_SL_i = Z_3x3;
            J_SL_i.block<3,1>(0,2) = J_SL3D_i.block<3,1>(0,2);

            // Extract rotation from estimated rover pose 
            OptionalJacobian<3,6>::Jacobian J_SR_j;
            const Rot3 rot_site_robot = site_robot.rotation(J_SR_j);

            // Compute relative rotation from local terrain frame 
            OptionalJacobian<3,3>::Jacobian J_LR_sl,J_LR_sr;
            const Rot3 rot_LR = traits<Rot3>::Between(rot_site_local,rot_site_robot,J_LR_sl,J_LR_sr);
            if (Hi) *Hi = J_LR_sl * J_SL_i;
            if (Hj) *Hj = J_LR_sr * J_SR_j;
            
            // SOMEHOW THIS IFDEF CREATES A RUNTIME ERROR. DON'T KNOW WHY. IT IS THE SAME AS THE ONE IN gtsam/slam/BetweenFactor.h and makes sense to me.
#ifdef SLOW_BUT_CORRECT_BINARYROTATIONFACTOR
            //OptionalJacobian<3,3>::Jacobian logJ;
            typename traits<Rot3>::ChartJacobian::Jacobian logJ;
            Vector error = traits<Rot3>::Local(measure_,rot_LR, boost::none, &logJ);
            if (Hi) *Hi = logJ * (*Hi);
            if (Hj) *Hj = logJ * (*Hj);
#else

            return traits<Rot3>::Local(measure_,rot_LR);

#endif
        }

        Vector TranslationFactor::evaluateError(const Pose2 &site_local, const Pose3 &site_robot,  boost::optional<Matrix&> H_sl, boost::optional<Matrix&> H_sr) const
        {
            // Create Pose3 from Pose2 to get the true site->local Pose 
            OptionalJacobian<3,3>::Jacobian J_Rot_sl;
            const Pose3 site_local3D(Rot3::Expmap(Vector3(0,0,site_local.theta()),J_Rot_sl),Point3(site_local.x(),site_local.y(),altitude_));

            // Compute Jacobian
            OptionalJacobian<6,3>::Jacobian J_sl = Eigen::Matrix<double,6,3>::Zero();
            J_sl.block<3,1>(0,2) = J_Rot_sl.block<3,1>(0,2);
            J_sl.block<2,2>(3,0) = I_2x2;

            // Compute Between transform to obtain LTF->Robot
            OptionalJacobian<6,6>::Jacobian J_LR_sl ,J_LR_sr;
            Pose3 robot1_robot2 = traits<Pose3>::Between(site_local3D,site_robot,J_LR_sl, J_LR_sr);

            // Take translation from the transform to obtain estimated translation
            OptionalJacobian<3,6>::Jacobian J_Trans_lr;
            Point3 T = robot1_robot2.translation(J_Trans_lr);

            // Update full Jacobian
            if (H_sl) *H_sl = J_Trans_lr * J_LR_sl * J_sl;
            if (H_sr) *H_sr = J_Trans_lr * J_LR_sr; 

            // Compute and return error
            Point3 error = T - measure_;
            return error;
        }

        gtsam::NonlinearFactor::shared_ptr TranslationFactor::clone() const 
        {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new TranslationFactor(*this))); 
        }

        Vector FullPoseFactor::evaluateError(const Pose2 &Ti, const Pose3 &Tj, boost::optional<Matrix&> Hi, boost::optional<Matrix&> Hj) const
        {
            OptionalJacobian<3,3>::Jacobian J;
            const Pose3 siteFrame_localFrame(Rot3::Expmap(Vector3(0,0,Ti.theta()),J),Point3(Ti.x(),Ti.y(),altitude_));
            OptionalJacobian<6,3>::Jacobian localFrameJ = Eigen::Matrix<double,6,3>::Zero();
            localFrameJ.block<3,1>(0,2) = J.block<3,1>(0,2);
            localFrameJ.block<2,2>(3,0) = I_2x2;

            OptionalJacobian<6,6>::Jacobian Hbi;
            const Pose3 estimatedPose = traits<Pose3>::Between(siteFrame_localFrame,Tj,Hbi,(Hj) ? Hj : boost::none );
            if (Hi) *Hi = Hbi*localFrameJ;

#ifdef SLOW_BUT_CORRECT_FULLPOSEFACTOR
            OptionalJacobian<6,6> errorDerivative;
            Pose3 error = traits<Pose3>::Local(measure_,estimatedPose, boost::none, (Hi || Hj)? errorDerivative : 0)
            if (Hi) *Hi = errorDerivative * Hi;
            if (Hj) *Hj = errorDerivative * Hj
            return error;
#else
            return traits<Pose3>::Local(measure_,estimatedPose);
#endif
        }



    }
}
