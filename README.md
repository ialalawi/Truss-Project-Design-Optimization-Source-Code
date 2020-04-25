# Truss-Project-Design-Optimization-Source-Code

I created code to optimize the design of a planar soda straw truss using MATLAB. The objective is to create a design that would yield a high maximum load and high load to cost ratio. 

General Code Description: 

In particular, two for-loops are used to create the coefficient matrix A, with the first loop
corresponding to Ax and the second to Ay, i.e. the top half and bottom half matrices concatenated
vertically to create A. In order to create the elements of A, the MATLAB built-in find() and size()
functions were used, in which the former returns joint-member connection indices, and the latter outputs
the number of rows and columns of the connection matrix for use in the loop to create matrix A; this is
because matrix A has the same number of rows but three less columns than the connection matrix, hence
the resultant matrix A is horizontally concatenated with the reaction force vectors. It is important to note
that the reshape() function was used to denote each column # as the member # and the two joints
connected to the member are found in each column; this allows easier indexing into the coefficient matrix.
In order to determine the member forces, typically inv() function is used, but that is less efficient,
according to MATLAB, than the ‘\’ inverse matrix operator, so ‘\’ is used instead. In order to determine
which members are under compression or tension, the program loops through the resultant member force
vector and uses nested if-else statements to determine and assign a character (either ‘T’ or ‘C’ or ‘’ if it’s
a zero-force member) to the ‘state’ of each member, i.e. whether or not it’s under compression or tension.
To calculate the scaling ratio, a separate file was made called ‘buckling.m’ which is a function that
outputs the buckling strength (without its uncertainty) of a given straw length using the straw lab data
analysis empirical fit. To calculate the maximum load, the max() function was used, where the maximum
value of the scaling ratio vector was found. In addition, and although not explicitly found as an output on
the code, the critical member is found using a combination of the find() and ismember() functions, where
the latter outputs the logical indices for which a member achieves the maximum scaling ratio, and its
index is found by inputting such a logical vector into the find() function.

For more specific code-related aspects, please download or view the .m file found in the repository. 
