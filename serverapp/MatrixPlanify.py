
def main():
    Rx = [['1', '0', '0'], 
    ['0', 'cosx', '-sinx'],
    ['0', 'sinx', 'cosx']]

    Ry = [['cosy', '0', 'siny'],
    ['0', '1', '0'],
    ['-siny', '0', 'cosy']]

    v = [['accel.ax'],
    ['accel.ay'],
    ['accel.az']]

    res = multiply(Rx, multiply(Ry, v))
    printMat("Z", res)

    rotation()


def rotation():
    P = [['P[0][0]', 'P[0][1]', 'P[0][2]', 'P[0][3]'], 
        ['P[1][0]', 'P[1][1]', 'P[1][2]', 'P[1][3]'],
        ['P[2][0]', 'P[2][1]', 'P[2][2]', 'P[2][3]'], 
        ['P[3][0]', 'P[3][1]', 'P[3][2]', 'P[3][3]']]

    # No x y relation
    # P = [['P[0][0]', '0', 'P[0][2]', '0'], 
    #     ['0', 'P[1][1]', '0', 'P[1][3]'],
    #     ['P[2][0]', '0', 'P[2][2]', '0'], 
    #     ['0', 'P[3][1]', '0', 'P[3][3]']]

    # No x y and derivate relation
    # P = [['P[0][0]', '0', '0', '0'], 
    #     ['0', 'P[1][1]', '0', '0'],
    #     ['0', '0', 'P[2][2]', '0'], 
    #     ['0', '0', '0', 'P[3][3]']]

    X = [['X[0]'], ['X[1]'], ['X[2]'], ['X[3]']]

    Xp = [['Xp[0]'], ['Xp[1]'], ['Xp[2]'], ['Xp[3]']]

    Y = [['Y[0]'], ['Y[1]'], ['Y[2]'], ['Y[3]']]

    U = [['U[0]'], ['U[1]']]

    A = [['1', '0', 'dt', '0'],
        ['0', '1', '0', 'dt'],
        ['0', '0', '1', '0'],
        ['0', '0', '0', '1']]

    B = [['0.5f*dt2', '0'],
        ['0', '0.5f*dt2'],
        ['dt', '0'],
        ['0', 'dt']]

    # Q = [['Q[0][0]', 'Q[0][1]', 'Q[0][2]', 'Q[0][3]'],
    #     ['Q[1][0]', 'Q[1][1]', 'Q[1][2]', 'Q[1][3]'],
    #     ['Q[2][0]', 'Q[2][1]', 'Q[2][2]', 'Q[2][3]'],
    #     ['Q[3][0]', 'Q[3][1]', 'Q[3][2]', 'Q[3][3]']]

    # # Adjusted for X and Y are not related
    # Q = [['Q[0][0]', '0', 'Q[0][2]', '0'],
    #     ['0', 'Q[1][1]', '0', 'Q[1][3]'],
    #     ['Q[2][0]', '0', 'Q[2][2]', '0'],
    #     ['0', 'Q[3][1]', '0', 'Q[3][3]']]

    # No x y and derivate relation
    Q = [['Q[0][0]', '0', '0', '0'],
        ['0', 'Q[1][1]', '0', '0'],
        ['0', '0', 'Q[2][2]', '0'],
        ['0', '0', '0', 'Q[3][3]']]

    # R = [['R[0][0]', 'R[0][1]', 'R[0][2]', 'R[0][3]'],
    #     ['R[1][0]', 'R[1][1]', 'R[1][2]', 'R[1][3]'],
    #     ['R[2][0]', 'R[2][1]', 'R[2][2]', 'R[2][3]'],
    #     ['R[3][0]', 'R[3][1]', 'R[3][2]', 'R[3][3]']]

    # Adjusted for X and Y are not related
    # R = [['R[0][0]', '0', 'R[0][2]', '0'],
    #     ['0', 'R[1][1]', '0', 'R[1][3]'],
    #     ['R[2][0]', '0', 'R[2][2]', '0'],
    #     ['0', 'R[3][1]', '0', 'R[3][3]']]

    # No x y and derivate relation
    R = [['R[0][0]', '0', '0', '0'],
        ['0', 'R[1][1]', '0', '0'],
        ['0', '0', 'R[2][2]', '0'],
        ['0', '0', '0', 'R[3][3]']]

    H = [['1', '0', '0', '0'], 
        ['0', '1', '0', '0'],
        ['0', '0', '1', '0'],
        ['0', '0', '0', '1']]

    H1 = [['1', '1', '1', '1'], 
        ['1', '1', '1', '1'],
        ['1', '1', '1', '1'],
        ['1', '1', '1', '1']]

    K = [['K[0][0]', 'K[0][1]', 'K[0][2]', 'K[0][3]'],
        ['K[1][0]', 'K[1][1]', 'K[1][2]', 'K[1][3]'],
        ['K[2][0]', 'K[2][1]', 'K[2][2]', 'K[2][3]'],
        ['K[3][0]', 'K[3][1]', 'K[3][2]', 'K[3][3]']]

    I = [['1', '0', '0', '0'], 
        ['0', '1', '0', '0'],
        ['0', '0', '1', '0'],
        ['0', '0', '0', '1']]

    Sinv = [['Sinv[0][0]', 'Sinv[0][1]', 'Sinv[0][2]', 'Sinv[0][3]'],
        ['Sinv[1][0]', 'Sinv[1][1]', 'Sinv[1][2]', 'Sinv[1][3]'],
        ['Sinv[2][0]', 'Sinv[2][1]', 'Sinv[2][2]', 'Sinv[2][3]'],
        ['Sinv[3][0]', 'Sinv[3][1]', 'Sinv[3][2]', 'Sinv[3][3]']]

    print("Xp = AX + BU:")
    AX = multiply(A, X)
    BU = multiply(B, U)
    printMat("Xp", add(AX, BU))

    

    print("")
    print("P = APAt + Q:")
    APAt = multiply(A, multiply(P,transpose(A)))
    #printMat("APAt", APAt)
    printMat("P", add(APAt, Q))    

    print("")
    print("K = PHt * inv(HPHt + R):")
    Pht = multiply(P, transpose(H))
    HPHtpR = add(multiply(H, multiply(P, transpose(H))),R)
    printMat("S", HPHtpR)
    printMat("K", multiply(Pht, Sinv))

    print("")
    print("Y = CY:")
    printMat("Y", Y)
    
    print("")
    print("X = Xp + K * (Y-Xp)")
    printMat("X", add(Xp, multiply(K, sub(Y, Xp))))

    print("")
    print("P = (I-KH) * P")
    printMat("P", multiply(sub(I, multiply(K, H1)), P))


def printMat(mat):
    for r in range(len(mat)):
        string = ""
        for c in range(len(mat[0])):
            string += ", "+mat[r][c]
        print(string[2:])

def printMat(dst, mat):
    for r in range(len(mat)):
        for c in range(len(mat[0])):
            if(len(mat[0]) > 1):
                print(dst + "[" + str(r) + "][" + str(c) + "] = "+mat[r][c])
            else:
                print(dst + "[" + str(r) + "] = "+mat[r][c])

def transpose(mat):
    mat_rows = len(mat)
    mat_cols = len(mat[0])

    result = []
    for i in range(mat_cols):
        result.append([])
    
    for col in range(mat_cols):
        for row in range(mat_rows):
            result[col].append(mat[row][col])

    return result

    

def multiply(mat_a, mat_b):
    a_rows = len(mat_a)
    if a_rows < 1:
        return []

    a_cols = len(mat_a[0])
    if a_cols < 1:
        return []

    b_rows = len(mat_b)
    if b_rows < 1:
        return []

    b_cols = len(mat_b[0])
    if b_cols < 1:
        return []

    for row in mat_a:
        if len(row) != a_cols:
            return []

    for row in mat_b:
        if len(row) != b_cols:
            return []


    if a_cols != b_rows:
        print("Dimensions don't match: %dx%d * %dx%d\n", a_rows, a_cols, b_rows, b_cols)
        return []

    result = []
    for i in range(a_rows):
        result.append([])
    for b_col in range(0, b_cols):
        for a_row in range(0, a_rows):
            value = ""
            for i in range(b_rows):
                if mat_a[a_row][i] == "0" or mat_b[i][b_col] == "0":
                    continue
                elif mat_a[a_row][i] == "1":
                    value += " + " + mat_b[i][b_col]
                elif mat_b[i][b_col] == "1":
                    value += " + " + mat_a[a_row][i]
                else:
                    value += " + " + mat_a[a_row][i] + " * " + mat_b[i][b_col]
                
            if value == "":
                result[a_row].append("0")
            else:
                result[a_row].append("("+value[3:]+")")
    return result

def divide(mat_a, mat_b):
    a_rows = len(mat_a)
    if a_rows < 1:
        return []

    a_cols = len(mat_a[0])
    if a_cols < 1:
        return []

    b_rows = len(mat_b)
    if b_rows < 1:
        return []

    b_cols = len(mat_b[0])
    if b_cols < 1:
        return []

    if a_rows != b_rows or a_cols != b_cols:
        return []

    result = []
    for row in range(a_rows):
        result.append([])
        for col in range(a_cols):
            result[row].append(mat_a[row][col] + " / " + mat_b[row][col])
    return result

def add(mat_a, mat_b):
    a_rows = len(mat_a)
    if a_rows < 1:
        return []

    a_cols = len(mat_a[0])
    if a_cols < 1:
        return []

    b_rows = len(mat_b)
    if b_rows < 1:
        return []

    b_cols = len(mat_b[0])
    if b_cols < 1:
        return []

    if a_rows != b_rows or a_cols != b_cols:
        return []

    result = []
    for row in range(a_rows):
        result.append([])
        for col in range(a_cols):
            if (mat_a[row][col] == '0'): 
               result[row].append(mat_b[row][col])
            elif mat_b[row][col] == '0':
                result[row].append(mat_a[row][col])
            else:
                result[row].append("(" + mat_a[row][col] + " + " + mat_b[row][col] + ")")
    return result

def sub(mat_a, mat_b):
    a_rows = len(mat_a)
    if a_rows < 1:
        return []

    a_cols = len(mat_a[0])
    if a_cols < 1:
        return []

    b_rows = len(mat_b)
    if b_rows < 1:
        return []

    b_cols = len(mat_b[0])
    if b_cols < 1:
        return []

    if a_rows != b_rows or a_cols != b_cols:
        return []

    result = []
    for row in range(a_rows):
        result.append([])
        for col in range(a_cols):
            if (mat_a[row][col] == '0'): 
                result[row].append("-" + mat_b[row][col])
            elif mat_b[row][col] == '0':
                result[row].append(mat_a[row][col])
            else:
                result[row].append("("+mat_a[row][col] + " - " + mat_b[row][col]+")")
            
    return result
            



if __name__ == "__main__":
    main()