from array import array
from functools import total_ordering
from posixpath import split
import sys
import numpy as np
import casadi
import forcespro
import forcespro.nlp
import os
import decimal
import config

def main():

    # REMINDER: Change mpc printlevel to 0 and optlevel to 2 or 3

    solver = forcespro.nlp.Solver.from_directory(os.path.join(os.path.dirname(__file__), "FORCESNLPsolver/"))

    N = config.N # Horizon length
    nvar = 6 # number of variables
    npar = 2 # number of parameter


    #                rho_sigma  track_width
    params = np.array([10000,        10.])
    all_parameters = np.transpose(np.tile(params,(1,N)))

    succes_counter = 0
    maxit_counter = 0
    fail_counter = 0
    total_counter = 0

    for v_init in np.arange(1., 19.1, 0.1):
        x0i = np.array([0., 0., 0., 0., 0., 0.]).reshape(nvar,1)
        x0 = np.transpose(np.tile(x0i, (1, N)))
        #                              e_y e_psi v   t
        xinit = np.transpose(np.array([0., 0.,   v_init, 0.]))
        problem = {"x0": x0,
                "xinit": xinit,
                "all_parameters": all_parameters}

        # Time to solve the NLP!
        output, exitflag, info = solver.solve(problem)
        if exitflag == 1:
            succes_counter += 1
            #    print("Success with: ", x0i[0][0], x0i[1][0], x0i[2][0], x0i[3][0], x0i[4][0])
            print("Success with v_init=", str(v_init)[0:4], " after ", info.solvetime)

        elif exitflag == 0:
            maxit_counter += 1
            print("Timeout with v_init=", str(v_init)[0:4])
        else:
            print("Failed with v_init=", str(v_init)[0:4], " With Exitflag=", exitflag)
            fail_counter += 1
        total_counter += 1


    print("success_counter = ", succes_counter)
    print("maxit_counter = ", maxit_counter)
    print("fail_counter = ", fail_counter)
    print("total_counter = ", total_counter)

    # print('Output:')
    # print("n\t\ta\t\t\tdelta\t\t\te_y\t\t\te_psi\t\t\tv\t\t\tt")
    # print("-"*144)

    # # To print without scientific notation
    # ctx = decimal.Context()
    # ctx.prec = 20

    # for x in output:
    #     line = "" + x + "\t"
    #     for num in output.get(x).tolist():
    #         y = ctx.create_decimal(repr(num))
    #         y = format(y, 'f')
    #         if(len(y)<16):
    #             y = y + ("0"*(16-len(y)))
    #         if(y[0]!='-'):
    #             y = "+" + y
    #         y = y[0:16]
    #         line = line + y + "\t"
    #     print(line)
    # print('\n')

    # print('Exitflag:')
    # print(exitflag) 
    # print('\n')

    # print('Info:')
    # print(' it: ' + str(info.it))
    # print(' res_eq: ' + str(info.res_eq))
    # print(' res_ineq: ' + str(info.res_ineq))
    # print(' rsnorm: ' + str(info.rsnorm))
    # print(' rcompnorm: ' + str(info.rcompnorm))
    # print(' pobj: ' + str(info.pobj))
    # print(' mu: ' + str(info.mu))
    # print(' solvetime: ' + str(info.solvetime))
    # print(' fevalstime: ' + str(info.fevalstime))
    # print('\n')

    # Make sure the solver has exited properly.
    # assert exitflag == 1, "bad exitflag"
    # sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
    #     .format(info.it, info.solvetime))


    #Doesnt work, model only in gen_script
    # Plot results and make interactive
    # ------------
    # extract output of solver
    # temp = np.zeros((np.max(model.nvar), model.N))
    # for i in range(0, model.N):
    #     temp[:, i] = output['x{0:02d}'.format(i+1)]
    # u = temp[0:2, :]
    # x = temp[2:6, :]

    # print(u)
    # print(x)

if __name__ == "__main__":
    main()
