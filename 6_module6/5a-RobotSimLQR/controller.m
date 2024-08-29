
function u = controller(params, t, X)
  % You have full state feedback available
  %u=0;
  K =  [0.003162277660167   0.402799313727944   0.003949686735209   0.044432616262353];
  u = K*X;
  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
end


