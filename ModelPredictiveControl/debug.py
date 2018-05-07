import pyomo.environ as env
# modified from https://groups.google.com/forum/#!topic/pyomo-forum/2364-6F0q8k


def CheckInstanceFeasibility(instance, tolerance):
    print()
    print ("*** Feasibility check:")
    infeasibility_found = False

    # for block in instance.block_data_objects():
    #     for con in block.component_data_objects(env.Constraint, active=True):
    #         resid = ComputeConstraintResid(con)
    #         if (resid > tolerance):
    #             infeasibility_found = True
    #             print (con.getname(True), con, resid)
    for block in instance.block_data_objects():
        for con in block.component_data_objects(env.Constraint, active=True):
            resid = ComputeConstraintResid_scaled(con)
            if (resid > tolerance):
                infeasibility_found = True
                print(con.getname(True), con, resid)

    if not infeasibility_found:
        print ("   No infeasibilities found.")
    print ("***")
    print ()


def ComputeConstraintResid(con):
    bodyval = env.value(con.body)
    upper_resid = 0
    if con.upper is not None:
        upper_resid = max(0, bodyval - env.value(con.upper))
    lower_resid = 0
    if con.lower is not None:
        lower_resid = max(0, env.value(con.lower) - bodyval)
    return  max(upper_resid, lower_resid)


# modified
def ComputeConstraintResid_scaled(con):
    bodyval = env.value(con.body)
    upper_resid = 0
    if con.upper is not None:
        upper_resid = max(0, bodyval - env.value(con.upper))
        if abs(env.value(con.upper)) > 1:
            upper_resid /= abs(env.value(con.upper))
    lower_resid = 0
    if con.lower is not None:
        lower_resid = max(0, env.value(con.lower) - bodyval)
        if abs(env.value(con.lower)) > 1:
            lower_resid /= abs(env.value(con.lower))
    return max(upper_resid, lower_resid)
