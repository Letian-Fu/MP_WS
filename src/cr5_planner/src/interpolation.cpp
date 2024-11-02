//
// Created by robert on 23-4-21.
//
#include "interpolation.h"
#include <time.h>
using namespace Eigen;

//nodes为节点向量，k为次数，i为第i个基函数，u为插值点位置
double basis_function(const std::vector<double>& nodes, int i, int k, double u) {
    if (k == 0) {
        if(u==1)
        {
            if (nodes[i] <= u && u <= nodes[i + 1]) {
                return 1.0;
            } else {
                return 0.0;
            }
        }
        else
        {
            if (nodes[i] <= u && u < nodes[i + 1]) {
                return 1.0;
            } else {
                return 0.0;
            }
        }
    }
    else {
            double alpha,beta;
            if (nodes[i + k] - nodes[i] == 0)
                alpha = 0;
            else
                alpha = (u - nodes[i]) / (nodes[i + k] - nodes[i]);

            if (nodes[i + k + 1] - nodes[i + 1] == 0)
                beta = 0;
            else
                beta = (nodes[i + k + 1] - u) / (nodes[i + k + 1] - nodes[i + 1]);
//        std::cout<<"i:"<<i<<" k:"<<k<<" u:"<<u<<" alpha: "<<alpha<<" beta: "<<beta<<std::endl;
            return alpha * basis_function(nodes, i, k - 1, u) + beta * basis_function(nodes, i + 1, k - 1, u);
        }
}

//创建3次B样条曲线的节点向量，Chord Length方法
std::vector<double> create_nodes(const std::vector<VectorXd> &data_points)
{
    std::vector<double> nodes;
    int dof=data_points[0].size();
    int n = data_points.size();
    for(int i=0;i<4;i++)
        nodes.push_back(0);
    std::vector<double> l;
    double L=0;
    for(int i=1; i<n;i++)
    {
        double dist;
        VectorXd temp;
        temp.resize(dof);
        temp=data_points[i]-data_points[i-1];
        dist = temp.norm();
        L+=dist;
        l.push_back(L);
    }
    for(int i=0;i<n-2;i++)
    {
        nodes.push_back(l[i]/l[n-2]);
    }
    for(int i=0;i<4;i++)
        nodes.push_back(1);
    return nodes;
}

//求解3次B样条曲线的控制点，nodes为节点向量,data_points数据点矩阵，包含的是dof维数据点
MatrixXd find_control_points(const std::vector<VectorXd> &data_points, std::vector<double> &nodes) {
//    for(double node : nodes)
//        std::cout<<node<<std::endl;
    int n = data_points.size();
    int dof = data_points[0].size();
//    std::cout<<"data_points_num: "<<n<<" nodes_num: "<<nodes.size()<<std::endl;
    MatrixXd node_matrix(n+2,n+2);
    node_matrix.setZero();
    node_matrix(0,0)=1;
    node_matrix(0,1)=-1;
    node_matrix(n+1,n)=-1;
    node_matrix(n+1,n+1)=1;
    node_matrix(1,1)=1;
    node_matrix(n,n)=1;
    for(int i=2;i<n;i++)
    {
        for(int j=0;j<3;j++)
            node_matrix(i,i+j-1)= basis_function(nodes,i+j-1,3,nodes[i+2]);
    }
//    std::cout<<"node_matrix:\n"<<node_matrix<<std::endl;
    //构造型值点矩阵
    MatrixXd data_matrix(n+2, dof);
    data_matrix.setZero();
    for(int i=0;i<n;i++)
    {
        data_matrix.row(i+1)=data_points[i];
    }
//    std::cout<<"data_matrix:\n"<<data_matrix<<std::endl;

    MatrixXd control_points =node_matrix.inverse()*data_matrix;
//    std::cout<<"control_points:\n"<<control_points<<std::endl;
    return control_points;
}

//data_points数据点矩阵是一个列数为6的矩阵，包含的是6维数据点,degree为次数
std::vector<VectorXd> interplote(const std::vector<VectorXd> &data_points, int inter_num, int dof) {
    clock_t start,finish;
    start=clock();
    std::vector<VectorXd> inter_points;
    std::vector<double> nodes = create_nodes(data_points);
    MatrixXd control_points = find_control_points(data_points, nodes);
    double n=data_points.size();

//    int exec_step = n + (n - 1) * inter_num;
//    inter_points.resize(exec_step);
    for (int i = 3; i < nodes.size()-4; i++) {
        double delta = (nodes[i + 1] - nodes[i]) / (inter_num + 1);
        for (int j = 0; j < inter_num + 1; j++) {
            double u=nodes[i]+j*delta;
            VectorXd temp(dof);
            temp.setZero();
            for(int k=0;k<n+2;k++)
            {
                for(int l=0;l<dof;l++)
                    temp(l)+= basis_function(nodes,k,3,u)*control_points(k,l);
            }
            inter_points.push_back(temp);
        }
    }
    inter_points.push_back(data_points[n-1]);
    finish=clock();
    double duration=(double)(finish - start) * 1000 / CLOCKS_PER_SEC;
//    std::cout<<"Interplote time: "<<duration<<" ms"<<std::endl;
    return inter_points;
}