#include <vector>
#include <random>
#include <Eigen/Dense>

class MVN{
    public:
        MVN(const Eigen::VectorXd& mu, const Eigen::MatrixXd& sigma):mu(mu),sigma(sigma){}

        /**
         * @brief Returns the pdf value of x for the multivariate normal distribution with mean mu and covariance sigma.
         * 
         * @param x : Value at which the pdf is evaluated.
         * @return double : pdf value for x.
         */
        double pdf(const Eigen::VectorXd& x) const;
        /**
         * @brief Generates a random vector from the multivariate normal distribution with mean mu and covariance sigma.
         * 
         * @return Eigen::VectorXd : The random vector.
         */
        Eigen::VectorXd rnd() const;
        /**
         * @brief Generates n random vectors from the multivariate normal distribution with mean mu and covariance sigma.
         * 
         * @param N : The number of random vectors to be generated.
         * @return Eigen::MatrixXd : The random vectors generated. Each random vector is stored in a column. The dimensions of the matrix is (n,N).
         */
        Eigen::MatrixXd rnd(const int N) const;

    private:
        // Methods
        Eigen::VectorXd generate_zero_mean_normal_vector() const;
        Eigen::MatrixXd square_root_of_covariance() const;
        // Variables
        const Eigen::VectorXd mu; // n-dimensional column vector which holds mean of the normal distribution.
        const Eigen::MatrixXd sigma; // (n,n) dimensional matrix which holds covariance of the normal distribution.
        // Constants
        const unsigned int SAMPLE_SIZE = 500; // The number of random numbers used to generate the distribution.
};