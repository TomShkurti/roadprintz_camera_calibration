function [U_D, V_D] = distort(U, V, D, P)
	hrez = P(1, 3);
	vrez = P(2, 3);
	
	U_small = (U - hrez) / (hrez * 2);
	V_small = (V - vrez) / (vrez * 2);
	
	k1 = D(1);
	k2 = D(2);
	k3 = D(3);
	p1 = D(4);
	p2 = D(5);
	
	r2 = U_small .^ 2 + V_small .^ 2;
	r4 = r2 * r2;
	r6 = r2 * r2 * r2;
	
	U_radial = U_small * (1 + k1*r2 + k2*r4 + k3*r6);
	V_radial = V_small * (1 + k1*r2 + k2*r4 + k3*r6);
	
	U_tangential = (2 * p1 * U_small * V_small + p2 * (r2 + 2 * U_small ^ 2));
	V_tangential = (p1 * (r2 + 2 *V_small ^ 2) + 2 * p2 *  U_small * V_small);
	
	U_D = ((U_radial + U_tangential) * hrez * 2) + hrez;
	V_D = ((V_radial + V_tangential) * vrez * 2) + vrez;
end