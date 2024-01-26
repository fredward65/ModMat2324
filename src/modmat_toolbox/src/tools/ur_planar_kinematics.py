#!/usr/bin/env python3

import numpy as np
import sympy as sym
from .command import PlanarArmCommander

class PlanarKinematicsCommander(PlanarArmCommander):
    def __init__(self) -> None:
        """
        Planar Kinematics Commander for a UR3 Arm
        """
        # Arm links dimensions
        super().__init__()
        self.link_lenghts:dict = {'l1': 0.1519, 'l2': 0.24365, 'l3': 0.21325, 'l4':  0.11235}
        l1 = self.link_lenghts["l1"]
        l2 = self.link_lenghts["l2"]
        l3 = self.link_lenghts["l3"]
        l4 = self.link_lenghts["l4"]
        self.links = [l1, l2, l3, l4]
        self.kin_solver = SimpleKinematicSolver()
        self.fk_solver = self.kin_solver.lambda_fk()
        self.ik_solver = self.kin_solver.lambda_ik()

    def forward_kinematics(self, joint_angles:np.ndarray) -> np.ndarray:
        """
        Forward Kinematics for a UR3 Arm
        """
        arm_pose = np.array(self.fk_solver(joint_angles, self.links))
        return arm_pose

    def inverse_kinematics(self, arm_pose:np.ndarray) -> np.ndarray:
        """
        Inverse Kinematics for a UR3 Arm
        """
        joint_angles = np.array(self.ik_solver(arm_pose, self.links))
        return joint_angles


def create_tf_matrix(theta:float, dx:float, dz:float) -> sym.Matrix:
        """
        Create a Transformation Matrix for the plane XZ
        """
        R = sym.Matrix(([sym.cos(theta),-sym.sin(theta)],\
                        [sym.sin(theta), sym.cos(theta)]))
        p = sym.Matrix(([dx],\
                        [dz]))
        A =  R.row_join(R @ p).col_join(sym.Matrix([[0,0,1]]))
        return A


class SimpleKinematicSolver(object):
    def __init__(self) -> None:
        """
        Simple Kinematics Solver for UR Arm in XZ planar configuration
        This solver uses SymPy Symbols and Solver methods
        """
        # Link lenghts 
        self.L1 = sym.Symbol('L1', real=True)
        self.L2 = sym.Symbol('L2', real=True)
        self.L3 = sym.Symbol('L3', real=True)
        self.L4 = sym.Symbol('L4', real=True)
        # Joint angles
        self.th_1 = sym.Symbol('th_1', real=True)
        self.th_2 = sym.Symbol('th_2', real=True)
        self.th_3 = sym.Symbol('th_3', real=True)
        # Cartesian pose variables
        self.x = sym.Symbol('x', real=True)
        self.z = sym.Symbol('z', real=True)
        self.th = sym.Symbol('th', real=True)
        self.forward_kinematics = self.compute_forward_kinematics()
        self.inverse_kinematics = self.compute_inverse_kinematics()

    def compute_forward_kinematics(self) -> list:
        """
        Generate Forward Kinematics expressions for UR Arm in XZ planar configuration
        """
        print("Computing Forward Kinematics lambda expression...")

        # Planar transformation matrices
        A_1 = create_tf_matrix(0        ,       0, self.L1)
        A_2 = create_tf_matrix(self.th_1, self.L2,       0)
        A_3 = create_tf_matrix(self.th_2, self.L3,       0)
        A_4 = create_tf_matrix(self.th_3, self.L4,       0)
        A = A_1 @ A_2 @ A_3 @ A_4
        # print("A : \n", A)

        pos_x = sym.trigsimp(A[0, -1])
        pos_z = sym.trigsimp(A[1, -1])
        theta = sym.trigsimp(sym.atan(A[1, 0] / A[0, 0]), inverse=True)
        arm_pose = [pos_x, pos_z, theta]

        return arm_pose

    def compute_inverse_kinematics(self) -> list:
        """
        Generate Inverse Kinematics expressions for UR Arm in XZ planar configuration
        """
        print("Computing Inverse Kinematics lambda expression...")
        
        eq_x  = self.x  - self.forward_kinematics[0]
        eq_z  = self.z  - self.forward_kinematics[1]
        eq_th = self.th - self.forward_kinematics[2]

        # Variable replacements for cos and sin functions
        c1 = sym.Symbol('c1')
        s1 = sym.Symbol('s1')
        c12 = sym.Symbol('c12')
        s12 = sym.Symbol('s12')
        cth = sym.Symbol('cth')
        sth = sym.Symbol('sth')

        # Equation solving and manipulation
        eq_1 = eq_x
        eq_2 = eq_z
        th_3 = sym.solve(eq_th, self.th_3)[0]

        var_subs_1 = {self.th_3: th_3,
                      sym.sin(self.th_1): s1,
                      sym.cos(self.th_1): c1,
                      sym.sin(self.th_1 + self.th_2): s12,
                      sym.cos(self.th_1 + self.th_2): c12}
        var_subs_2 = {sym.sin(self.th): sth,
                      sym.cos(self.th): cth,
                      self.z - self.L1: self.z}
        eq_1 = eq_1.subs(var_subs_1).subs(var_subs_2)
        eq_2 = eq_2.subs(var_subs_1).subs(var_subs_2)
        eq_3 = c1**2 + s1**2 - 1
        eq_4 = c12**2 + s12**2 - 1
        
        sol = sym.solve([eq_1, eq_2, eq_3, eq_4],
                        [s1, c1, s12, c12], dict=True)
        th_1 = sym.atan2(sol[0][s1], sol[0][c1])
        th_2 = sym.atan2(sol[0][s12], sol[0][c12]) - th_1

        sub_vars = {self.z: self.z - self.L1,
                    sth: sym.sin(self.th),
                    cth: sym.cos(self.th)}
        th_1 = th_1.subs(sub_vars)
        th_2 = th_2.subs(sub_vars)
        th_3 = th_3.subs({self.th_1: th_1, self.th_2: th_2})

        joint_pose = [th_1, th_2, th_3]
        return joint_pose

    def lambda_fk(self):
        """
        Return Forward Kinematics model as a numpy compatible lambda expression
        """
        return sym.lambdify(([self.th_1, self.th_2, self.th_3],
                             [self.L1, self.L2, self.L3, self.L4]),
                            self.forward_kinematics, modules="numpy")
    
    def lambda_ik(self):
        """
        Return Inverse Kinematics model as a numpy compatible lambda expression
        """
        return sym.lambdify(([self.x, self.z, self.th],
                             [self.L1, self.L2, self.L3, self.L4]),
                            self.inverse_kinematics, modules="numpy")


def main():
    kin_solver = SimpleKinematicSolver()
    print(kin_solver.forward_kinematics)
    print(kin_solver.inverse_kinematics)


if __name__ == '__main__':
    main()
