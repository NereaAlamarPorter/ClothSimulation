using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

public class Spring : MonoBehaviour {

    #region InEditorVariables

    public float Stiffness;
    public float Damping;
    public Node nodeA;
    public Node nodeB;

    #endregion

    public enum SpringType { Stretch, Bend };
    public SpringType springType;

    public float Length0;
    public float Length;
    public Vector3 dir;

    private PhysicsManager Manager;

    public Spring(Node a, Node b, SpringType s)
    {
        nodeA = a;
        nodeB = b;
        springType = s;
    }

    // Update is called once per frame
    void Update () {
    }

    // Use this for initialization
    public void Initialize(float stiffness, float damping, PhysicsManager m)
    {
        // TO BE COMPLETED
        Stiffness = stiffness;
        Damping = damping;
        Manager = m;
        UpdateState();
        Length0 = Length;
    }

    // Update spring state
    public void UpdateState()
    {
        dir = nodeA.Pos - nodeB.Pos;
        Length = dir.magnitude;
        dir = (1.0f / Length) * dir;
    }

    // Compute force and add to the nodes
    public void ComputeForce()
    {
        // TO BE COMPLETED
        MatrixXD vector = MatrixXD.Build.Dense(3, 1);
        MatrixXD vectorT = MatrixXD.Build.Dense(1, 3);
        MatrixXD F = MatrixXD.Build.Dense(3, 1);
        MatrixXD u = MatrixXD.Build.Dense(3, 1);
        MatrixXD uT = MatrixXD.Build.Dense(1, 3);

        nodeA.Force += -Stiffness * (Length - Length0) * (nodeA.Pos - nodeB.Pos) / Length;
        nodeB.Force += -Stiffness * (Length - Length0) * (nodeB.Pos - nodeA.Pos) / Length;

        //Fa
        for (int i = 0; i < 3; i++)
            u[i, 0] = (nodeA.Pos[i] - nodeB.Pos[i]) / Length;

        for (int i = 0; i < 3; i++)
            uT[0, i] = u[i, 0];

        for (int i = 0; i < 3; i++)
            vector[i, 0] = nodeA.Vel[i] - nodeB.Vel[i];

        F = -Damping * u*uT*vector;
        for (int i = 0; i < 3; i++)
        {
            nodeA.Force[i] += (float)F[i, 0];
        }

        //Fb
        for (int i = 0; i < 3; i++)
            u[i, 0] = (nodeB.Pos[i] - nodeA.Pos[i]) / Length;

        for (int i = 0; i < 3; i++)
            uT[0, i] = u[i, 0];

        for (int i = 0; i < 3; i++)
            vector[i, 0] = nodeB.Vel[i] - nodeA.Vel[i];

        F = -Damping * u * uT * vector;
        for (int i = 0; i < 3; i++)
        {
            nodeB.Force[i] += (float)F[i, 0];
        }
        
    }

    // Get Force Jacobian
    public void GetForceJacobian(MatrixXD dFdx, MatrixXD dFdv)
    {
        // TO BE COMPLETED
        MatrixXD u = MatrixXD.Build.Dense(3, 1);
        MatrixXD uT = MatrixXD.Build.Dense(1, 3);
        MatrixXD duxa = MatrixXD.Build.Dense(3, 3);
        MatrixXD dlxa = MatrixXD.Build.Dense(1, 3);
        MatrixXD I = MatrixXD.Build.DenseIdentity(3, 3);
        MatrixXD faxa, faxb, fbxb, fbxa, fava, favb, fbvb, fbva;


        for (int i = 0; i < 3; i++)
            u[i, 0] = (nodeA.Pos[i] - nodeB.Pos[i]) / Length;

        for (int i = 0; i < 3; i++)
            uT[0, i] = u[i, 0];

        dlxa = uT;
        duxa = (I - u * uT) / Length;

        faxa = -Stiffness * ((Length - Length0) * duxa + u * dlxa);
        fbxb = faxa;
        fbxa = -faxa;
        faxb = -fbxb;

        dFdx.SetSubMatrix(0, 0, faxa);
        dFdx.SetSubMatrix(0, 1, faxb);
        dFdx.SetSubMatrix(1, 0, fbxa);
        dFdx.SetSubMatrix(1, 1, fbxb);

        fava = -Damping * u * uT;
        fbvb = fava;
        fbva = -fava;
        favb = -fbvb;

        dFdv.SetSubMatrix(0, 0, fava);
        dFdv.SetSubMatrix(0, 1, favb);
        dFdv.SetSubMatrix(1, 0, fbva);
        dFdv.SetSubMatrix(1, 1, fbvb);

    }

}
