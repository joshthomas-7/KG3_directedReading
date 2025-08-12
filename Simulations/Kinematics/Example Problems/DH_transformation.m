function A = DH_transformation(a,alpha,d,theta)
%%% C3376353
%%% Joshua Thomas    
    A = [cos(theta) -round(cos(alpha))*sin(theta) round(sin(alpha))*sin(theta) a*cos(theta);
         sin(theta) -round(cos(alpha))*cos(theta) -round(sin(alpha))*cos(theta) a*sin(theta);
         0 round(sin(alpha)) round(cos(alpha)) d;
         0 0 0 1];
end