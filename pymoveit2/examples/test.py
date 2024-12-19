ls = []
var = {
    "value": [1,2,3]
}

var2 = [3]
# ls.append(var["value"])
ls.append(var["value"])
print(ls)

var["value"][1] = 3
var["value"].pop(0)
var2[0] = 4
print(ls)

str1 = "boj_3"
# ls1 = str1.split("_")
ls1 = str1.strip("boj_")
print(ls1)

ls2 = [9]*9
print(ls2)

print(not 0)