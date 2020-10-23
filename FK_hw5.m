function T = FK_hw5(q,s,L)
T = Rz(q)*Tx(s)*Tx(L)*Tx(L);
end
