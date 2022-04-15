# Prints out the position and color for a path for the robot that spells out "H"
# Authors: Joseph Gioia & Prithve Shekar

def main():
    z_min = 0
    z_max = 100
    z_mid = (z_min + z_max) // 2

    color_pairs = []
    print("s = [", end="")

    # H
    color = 1
    num_increments = 0
    x = 1500
    y_min = 0
    y_max = 100

    # H left upstroke
    num_increments += printLine(x, x, y_min, y_min, z_min, z_max, 1)

    # H left downstroke
    num_increments += printLine(x, x, y_min, y_min, z_max, z_mid, 1)

    # H crossstroke
    num_increments += printLine(x, x, y_min, y_max, z_mid, z_mid, 1)

    # H right upstroke
    num_increments += printLine(x, x, y_max, y_max, z_mid, z_max, 1)

    # H right downstroke
    num_increments += printLine(x, x, y_max, y_max, z_max, z_min, 1)

    color_pairs.append((color, num_increments))

    print("]")

    print("c = [", end="")
    printColor(color_pairs)
    print("]")

def printLine(x0, y0, z0, x1, y1, z1, max_increment):
    x = x0
    y = y0
    z = z0

    x_dist = abs(x1 - x0)
    y_dist = abs(y1 - y0)
    z_dist = abs(z1 - z0)

    x_increment = 0
    y_increment = 0
    z_increment = 0
    num_increments = 0

    if x_dist >= y_dist and x_dist >= z_dist:
        num_increments = x_dist // max_increment
    elif y_dist >= x_dist and y_dist >= z_dist:
        num_increments = y_dist // max_increment
    else:
        num_increments = z_dist // max_increment

    x_increment = (x1 - x0) / num_increments
    y_increment = (y1 - y0) / num_increments
    z_increment = (z1 - z0) / num_increments

    for i in range(num_increments):
        print("[" + str(int(x)) + ";" + str(int(y)) + ";" + str(int(z)) + "],", end="")
        x += x_increment
        y += y_increment
        z += z_increment
    
    print("[" + str(int(x1)) + ";" + str(int(y1)) + ";" + str(int(z1)) + "],", end="")

    return num_increments + 1

def printColor(color_pairs):
    for color_pair in color_pairs:
        for i in range(color_pair[1]):
            print(str(color_pair[0]) + ",", end="")

if __name__=="__main__":
    main()
