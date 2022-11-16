// #ifndef DORY_EKF_H
// #define DORY_EKF_H

// #include <Eigen/Dense>
// using namespace Eigen;

// namespace DoryLoc {
//     class EKF {
//     public:

//         Vector4d x;
//         Matrix4d P;
//         Matrix4d Q;
//         Matrix4d R;

//         /**
//          * @param initState The initial state vector {x, y, z, yaw}
//          * @param actionCovariance The covariance matrix for the action with the 
//          * independent noise for each state variable on the diagonal in order {x, y, z, yaw}
//          * @param measurementCovariance The covariance matrix for the measurement 
//          * with the independent noise for each state variable on the diagonal in 
//          * order {x, y, z, yaw} 
//         */
//         EKF(Vector4d initState, Matrix4d actionCovariance, Matrix4d measurementCovariance) 
//         : x(initState)
//         , Q(actionCovariance)
//         , R(measurementCovariance)
//         {

//         }

//     void predict(Vector4d uk) {
        
//         R = np.array([[np.cos(self.xk[2]), -np.sin(self.xk[2])],[np.sin(self.xk[2]), np.cos(self.xk[2])]])

//         uk[:2] = np.dot(R,uk[:2])
        
//         self.xk += uk

//         cTh = math.cos(uk[2])
//         sTh = math.sin(uk[2])
//         // note this x and y is in global ref frame
//         x = uk[0]
//         y = uk[1]
//         A = np.eye(3) // type: np.ndarray
//         A[0,2] = -sTh * x - cTh * y
//         A[1,2] = cTh * x - sTh * y

//         W = np.eye(3) // type: np.ndarray
//         W[0,0] = cTh
//         W[0,1] = -sTh
//         W[1,0] = sTh
//         W[1,1] = cTh

//         self.Pk = A.dot(self.Pk).dot(A.transpose()) + W.dot(self.Qk).dot(W.transpose())

//     void data_association() {}

//     void update_position(Hk_list, Vk_list, Sk_list, Rk_list){
//         // Compose list of matrices as single matrices
//         n = len(Hk_list)
//         H = np.zeros((2 * n, 3))
//         v = np.zeros((2 * n))
//         S = np.zeros((2 * n, 2 * n))
//         R = np.zeros((2 * n, 2 * n))
//         for i in range(n{
//             H[2 * i:2 * i + 2, :] = Hk_list[i]
//             v[2 * i:2 * i + 2] = Vk_list[i]
//             S[2 * i:2 * i + 2, 2 * i:2 * i + 2] = Sk_list[i]
//             R[2 * i:2 * i + 2, 2 * i:2 * i + 2] = Rk_list[i]

//         // There is data to update
//         if not n > 0:
//             return

//         // TODO: Program this void
//         ################################################################
//         // Do the EKF update
//         // we are assuming v and S have been calculated correctly already
//         // v = z - h
//         S = H.dot(self.Pk).dot(H.T) + R
//         K = self.Pk.dot(H.T).dot(np.linalg.pinv(S))
//         self.xk += K.dot(v)
//         // I - K*H
//         ikh = np.eye(3) - K.dot(H)
//         self.Pk = ikh.dot(self.Pk).dot(ikh.T) + K.dot(R).dot(K.T)
//         print("\n=== update shapes ===\nP: %s\nH: %s\nK: %s\nS: %s\n" % (self.Pk.shape, H.shape, K.shape, S.shape))

//     // ==========================================================================
//     void jacobianH(self, lineworld, xk{
//         """
//         Compute the jacobian of the get_polar_line void.

//         It does it with respect to the robot state xk (done in pre-lab).
//         """
//         // TODO: Program this void
//         ################################################################
//         // Complete the Jacobian H from the pre-lab
//         // Jacobian H
//         H = np.zeros((2, 3))
//         // H = np.zeros
//         x,y,th = xk
//         r_w, th_w = lineworld
//         g = atan2(y,x) - th_w // gamma
//         if x == 0: x += 1e-6
//         x2 = x**2
//         y2 = y**2
//         d = sqrt(x2 + y2)
//         yx2 = (y/x)**2
//         H[0,0] = -(
//           ( y*sin(g) + x*cos(g) ) / d
//         )
//         H[0,1] = -(
//           ( y*cos(g) - x*sin(g) ) / d
//         )
//         H[1,2] = -1
//         return H



//     };
// }

// #endif