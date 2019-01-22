#include <infuse_pom_tokamak/factors/RelativeMeasure.hpp>


namespace tokamak
{
    namespace RelativeMeasure
    {
        Vector TranslationFactor::evaluateError(const Pose3 &site_robot1, const Pose3 &site_robot2,  boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const
        {
            OptionalJacobian<6,6>::Jacobian J_R1R2_sr1,J_R1R2_sr2;
            Pose3 robot1_robot2 = traits<Pose3>::Between(site_robot1,site_robot2,J_R1R2_sr1, J_R1R2_sr2);
            OptionalJacobian<3,6>::Jacobian J_Trans_r1r2;
            Point3 robot1_robot2_trans = robot1_robot2.translation(J_Trans_r1r2);

            if (H1) *H1 = J_Trans_r1r2 * J_R1R2_sr1;
            if (H2) *H2 = J_Trans_r1r2 * J_R1R2_sr2;

            Point3 error = robot1_robot2_trans - measure_;
            return error;

        }

        gtsam::NonlinearFactor::shared_ptr TranslationFactor::clone() const
        {
            return boost::static_pointer_cast<gtsam::NonlinearFactor>(
                    gtsam::NonlinearFactor::shared_ptr(new TranslationFactor(*this)));
        }
    }
}
