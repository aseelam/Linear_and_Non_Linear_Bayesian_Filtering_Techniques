#include <mvn.h>
#include <fstream>

int main(){
    Eigen::VectorXd mu(2);
    Eigen::MatrixXd sigma(2,2);
    mu << 10,10;
    sigma << 10, 3, 3, 5;
    MVN mvn(mu,sigma);
    Eigen::MatrixXd rnd_vector_list = mvn.rnd(2000);
    std::ofstream file("/home/aseelam/Desktop/rand_matrix.csv");
    auto num_rnd_vectors = rnd_vector_list.cols();
    for (int i =0; i< num_rnd_vectors;++i){
        file << rnd_vector_list.col(i)[0] << "," << rnd_vector_list.col(i)[1] << "," << mvn.pdf(rnd_vector_list.col(i)) << "\n";
    }
    file.close();
    return 0;
}