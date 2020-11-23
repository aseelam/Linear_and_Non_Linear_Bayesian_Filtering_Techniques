#include <mvn.h>

double MVN::pdf(const Eigen::VectorXd &x) const
{
    assert(x.rows() == mu.rows() && "The dimensions of the vectors do match");

    auto n = x.rows();
    double sqrt_2pi = std::sqrt(2 * M_PI);
    double quadratic_form = (x - mu).transpose() * sigma.inverse() * (x - mu);
    double norm_factor = std::pow(sqrt_2pi, -n) * std::pow(sigma.determinant(), -0.5);

    return norm_factor * exp(-0.5 * quadratic_form);
}

Eigen::VectorXd MVN::generate_zero_mean_normal_vector() const
{
    auto n = mu.rows();
    // Generate x from the N(0,I) distribution.
    Eigen::VectorXd x(n);
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(n);
    for (unsigned int i = 0; i < SAMPLE_SIZE; ++i)
    {
        x.setRandom();
        x = 0.5 * (x + Eigen::VectorXd::Ones(n));
        sum += x;
    }
    sum = sum - (static_cast<double>(SAMPLE_SIZE) / 2) * Eigen::VectorXd::Ones(n);
    x = sum / (std::sqrt(static_cast<double>(SAMPLE_SIZE) / 12));
    return x;
}

Eigen::MatrixXd MVN::square_root_of_covariance() const
{
    auto n = mu.rows();
    Eigen::MatrixXd sqrt_sigma(n, n);
    Eigen::LLT<Eigen::MatrixXd> chol_solver(sigma);

    // Cholesky decomposition succeeds only when matrix is positive definite.
    // 'sigma' however is guaranteed to be symmetric and positive semi-definite but not positive definite.
    // If it fails, it is assumed to be a positive semi-definite matrix and hence eigen solver is the best alternative.
    if (chol_solver.info() == Eigen::Success)
    {
        // Use cholesky solver
        sqrt_sigma = chol_solver.matrixL();
    }
    else
    {
        // Use eigen solver
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(sigma);
        sqrt_sigma = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
    }
    return sqrt_sigma;
}

Eigen::VectorXd MVN::rnd() const
{
    Eigen::VectorXd x = generate_zero_mean_normal_vector();
    Eigen::MatrixXd sqrt_sigma = square_root_of_covariance();
    Eigen::MatrixXd rnd_vector = sqrt_sigma * x + mu;
    return rnd_vector;
}

Eigen::MatrixXd MVN::rnd(const int N) const
{
    auto n = mu.rows();
    Eigen::MatrixXd rnd_vector_matrix(n, N); // Each column contains a random vector.
    for (int i = 0; i < N; ++i) rnd_vector_matrix.col(i) = generate_zero_mean_normal_vector();
    Eigen::MatrixXd sqrt_sigma = square_root_of_covariance();
    for (int i = 0; i < N; ++i) rnd_vector_matrix.col(i) = sqrt_sigma * rnd_vector_matrix.col(i) + mu;
    return rnd_vector_matrix;
}