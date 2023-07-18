# Mypaper-fast-calculation
Appendix, data and code of my papers

Toolbox needed in the MATLAB platform:
Yalmip
CPLEX
MPT3

The introduction of the data and code:
The upper layer: LP.m
The lower layer: HI_RCUC.m; ccy_test_UC_case30.m
The curve amending strategy: ccy_test_UC_case30.m
The renewable data: w-ref.m; 
The load data: excel2017.xls;
The thermal units data: excel2017.xls 

The process of the proposed two-layer solution method is as follow:
(1) Run HI_RCUC.m iteratively to obtain the commitment status, which can be verified by the comparison of the benchmark ccy_test_UC_case30.m.
(2) Fixed the commitment status, run LP.m to obtain the rough curve. The obtained curve would be of two types: with two vertices or with one vertex. The analysis is discussed detailed in the paper. For two vertices, obtain the critical vertices; for another, obtain the criticle vertex as the paper introduced.
(3) For the undetermined regions introduced in the paper, run ccy_test_UC_case30.m at each fixed step to amend the curve.
(4) In the new critical vertex, return to (1), update the commitment ststus, and repeat the above step to obtain the final relation curve. 

The function of the main file:
1) LP.m is to characterize the relationship curve between renewable penetration and desired energy storage size/the total cost.
2) ccy_test_UC_case30.m is the benchmark of the lower layer which is to solve the MILP problem
3) HI_RCUC.m is the relaxed clustered unit commitment model which is a linear programming problem.

Introduction of other files:
The commitment status u is obtained restored in ('MILP_result','u_double').
The parameter space of renewable penetration rate beta is in the interval [0,0.7].
The critical verterx can be found in the result of the upper lower obviously.
The capacity of renewable units is fixed. The maximum of the renewable outpput is equal to the coefficient times the renewable capacity. The coefficient is w-ref.m.
The data curve of 4 typical days is load_case30_4days.docx.


 
