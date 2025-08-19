function C = KG3_symbolicCalcs()

    %% Creating a symbolic version of the mass matrix for the Coriolis calcs
    myKG3_sym = KG3_sym();
    myKG3_sym = myKG3_sym.MassMatrix();

    %% Computing the Coriolis matrix
    myKG3_sym = myKG3_sym.CoriolisMatrix();

    %% Outputs
    M = myKG3_sym.M;
    C = myKG3_sym.C;

end