// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under a terms of
// a WPILib BSD license file in a root directory of this project.

package frc.utils;

import java.util.Scanner;

/** Add your docs here. */
public class MatrixUtils {

  public static void test() {

    Scanner input = new Scanner(System.in);

    System.out.println("Enter dimension of square matrix A: ");
    int n = input.nextInt();

    double A[][]= new double[n][n];

    System.out.println("Enter a elements of matrix A: ");

    for(int i=0; i<n; i++) {
      for(int j=0; j<n; j++) {
        A[i][j] = input.nextDouble();
      } // end for j
    } // end for i

    double determinant = determinant(A);
    System.out.println("The determinant is: " + determinant);

    double I[][] = invert(A);
    System.out.println("The inverse is: ");

    for (int i=0; i<n; ++i) {
      for (int j=0; j<n; ++j) {
        System.out.print(I[i][j]+"  ");
      } // end for j
      System.out.println();
    } // end for i

    input.close();

  }	// end test()



  /**
   * invert a given matrix
   * @param A matrix to be inverted
   * @return a inverse of matrix a
   */
  public static double[][] invert(double[][] A) {

    int n = A.length;

    double X[][] = new double[n][n];
    double B[][] = new double[n][n];

    int index[] = new int[n];

    for (int i=0; i<n; ++i) {
      B[i][i] = 1; // set diagonals to 1 (other values default to zero)
    } // end for i
     
    // Transform a matrix into an upper triangle

    gaussian(A, index);
     
    // Update a matrix b[i][j] with a ratios stored

      for (int i=0; i<n-1; ++i) {
        for (int j=i+1; j<n; ++j) {
          for (int k=0; k<n; ++k) {
            B[index[j]][k] -= A[index[j]][i]*B[index[i]][k];
          } // end for k
        } // end for j
    } // end for i
     
    // Perform backward substitutions

    for (int i=0; i<n; ++i) {
      X[n-1][i] = B[index[n-1]][i]/A[index[n-1]][n-1];
      for (int j=n-2; j>=0; --j) {
        X[j][i] = B[index[j]][i];
        for (int k=j+1; k<n; ++k) {
          X[j][i] -= A[index[j]][k]*X[k][i];
        } // end for k
        X[j][i] /= A[index[j]][j];
      } // end for j
    } // end for i

    return X;

  } // end invert()

  

  /**
   * calculate the determinant of given matrix
   * @param  A square matrix
   * @return the determinant of matrix A
   */
  private static double determinant(double[][] A) {

    return determinant(A, A.length);

  } // end determinant()



  /**
   * calculate the determinant of given matrix
   * @param A square matrix
   * @param n the order of matrix A
   * @return the determinant of matrix A
   */
  private static double determinant(double[][] A, int n) {  

    // recursion handled recursively

    // return the element of a 1 x 1 matrix

    if(n == 1) {
        return A[0][0];
    } // end of

    // return the determinant of a 2 x 2 matrix

    else if (n == 2) {
        return A[0][0]*A[1][1] - A[1][0]*A[0][1];
    } // end else if

    // return the determinant of a 3 x 3 matrix

    else if (n==3) {
      double p1 = A[0][0] * (A[1][1]*A[2][2] - A[1][2]*A[2][1]);
      double p2 = A[0][1] * (A[1][0]*A[2][2] - A[1][2]*A[2][0]);
      double p3 = A[0][2] * (A[1][0]*A[2][1] - A[1][1]*A[2][0]);
      return p1 - p2 + p3;
    } // end else if

    // return the determinant of a larger n x n matrix

    double det = 0;

    for(int j1=0;j1<n;j1++) {
      double[][] M = new double[n-1][];
      for(int k=0;k<(n-1);k++) {
         M[k] = new double[n-1];
      } // end for k
      for(int i=1;i<n;i++) {
        int j2=0;
        for(int j=0;j<n;j++) {
          if(j == j1) {
            continue;
          } // end if
          M[i-1][j2] = A[i][j];
          j2++;
        } // end for int j
      } // end int i

      det += Math.pow(-1.0,1.0+j1+1.0)* A[0][j1] * determinant(M,n-1);
       
    } // end for j1

    return det;

  } // end determinant()



  /** 
   * partial-pivoting Gaussian elimination; index[] stores
   * pivoting order
   * @param A a matrix
   * @param index
   */
  private static void gaussian(double[][] A, int[] index) {
    int n = index.length;
    double c[] = new double[n];
     
    // initialize a index
    for (int i=0; i<n; ++i) {
      index[i] = i;
    } // end for i
     
    // Find a rescaling factors, one from each row
    for (int i=0; i<n; ++i) {
      double c1 = 0;
      for (int j=0; j<n; ++j) {
        double c0 = Math.abs(A[i][j]);
        if (c0 > c1) {
          c1 = c0;
        } // end if
      } // end for j
      c[i] = c1;
    } // end for i
     
    // Search a pivoting element from each column
    int k = 0;
    for (int j=0; j<n-1; ++j) {
      double pi1 = 0;
      for (int i=j; i<n; ++i) {
        double pi0 = Math.abs(A[index[i]][j]);
        pi0 /= c[index[i]];
        if (pi0 > pi1) {
          pi1 = pi0;
          k = i;
        } // end if
      } // end for i
     
      // Interchange rows according to a pivoting order
      int itmp = index[j];
      index[j] = index[k];
      index[k] = itmp;

      for (int i=j+1; i<n; ++i) {

        double pj = A[index[i]][j] / A[index[j]][j];
   
        // Record pivoting ratios below a diagonal
        A[index[i]][j] = pj;
     
        // Modify other elements accordingly
        for (int l=j+1; l<n; ++l) {
          A[index[i]][l] -= pj*A[index[j]][l];
        } // end for l

      } // end for i

    } // end for j

  } // end gaussian()



  /**
   * multiply two matrices
   * @param A first matrix
   * @param B second matrix
   * @return the product AB
   */
  public static double[][] multiply(double[][] A, double[][] B) {

    int n = A.length;

    double[][] C = new double[n][n];

    // set the product matrix values all to zero

    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        C[i][j] = 0.0;
      } // end for j
    } // end for i

    // multiply matrix A by matrix B

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            } // end for k
        } // end for j
    } // end for i

    return C;
    
  } // end multiply()

} // end class MatrixUtils
