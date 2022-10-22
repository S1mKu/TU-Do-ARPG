
import sys
import numpy as np
import forcespro
import forcespro.nlp
import decimal
import config
import os
import forcespro
import forcespro.nlp

def main():
    # get solver
    solver = forcespro.nlp.Solver.from_directory(os.path.join(os.path.dirname(__file__), "FORCESNLPsolver/"))

    N = config.N # Horizon length
    nvar = 7 # number of variables
    npar = 2 # number of parameter

    # Simulation
    # ----------
    # Set initial guess to start solver from
    # [D, delta, e_y, e_psi, v, t]?
    # x0i = np.array([1.,0.,0.,0.,10.,10.]).reshape(nvar,1)

    # Everything on 0 works
    x0i = np.array([0.,0.,0.,0.,0.,0.,0.]).reshape(nvar,1)
    
    
    x0 = np.transpose(np.tile(x0i, (1, N)))
    # xinit = np.transpose(np.array([0.,0.,10.,0.]))
    # Start with only a little speed
    #                              e_y e_psi  v  delta  t
    xinit = np.transpose(np.array([0.,  0.,  1.,   0.,  0.]))

    # Set runtime parameters
    # p[0] list of rho_sigma
    # p[1] list of track width
    # p[2] list of lb of obstacles
    # p[3] list of ub ob obstacles
    # p[4] safety distacne
    # p[5] Tref
    #                rho_sigma  track_width
    params = np.array([10000,        2.,       0.,    10.])
    # params = np.array([10000,        10.]).reshape(npar,1)  # does not work with reshaping it
    all_parameters = np.transpose(np.tile(params,(1,N)))


    problem = {"x0": x0,
            "xinit": xinit,
            "all_parameters": all_parameters}

    # Time to solve the NLP!
    output, exitflag, info = solver.solve(problem)

    print('Output:')
    print("n\t\ta\t\t\tdelta\t\t\te_y\t\t\te_psi\t\t\tv\t\t\tdelta\t\t\tt")
    print("-"*168)

    # To print without scientific notation
    ctx = decimal.Context()
    ctx.prec = 20

    for x in output:
        line = "" + x + "\t"
        for num in output.get(x).tolist():
            y = ctx.create_decimal(repr(num))
            y = format(y, 'f')
            if(len(y)<16):
                y = y + ("0"*(16-len(y)))
            if(y[0]!='-'):
                y = "+" + y
            y = y[0:16]
            line = line + y + "\t"
        print(line)
    print('\n')

    print('Exitflag:')
    print(exitflag) 
    print('\n')

    print('Info:')
    print(' it: ' + str(info.it))
    print(' res_eq: ' + str(info.res_eq))
    print(' res_ineq: ' + str(info.res_ineq))
    print(' rsnorm: ' + str(info.rsnorm))
    print(' rcompnorm: ' + str(info.rcompnorm))
    print(' pobj: ' + str(info.pobj))
    print(' mu: ' + str(info.mu))
    print(' solvetime: ' + str(info.solvetime))
    print(' fevalstime: ' + str(info.fevalstime))
    print('\n')

    # Make sure the solver has exited properly.
    assert exitflag == 1, "bad exitflag"
    sys.stderr.write("FORCESPRO took {} iterations and {} seconds to solve the problem.\n"\
        .format(info.it, info.solvetime))


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
