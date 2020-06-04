n = int(input())
res = 0
for i in range(n):
    a = int(input())
    if (a%5==0) and (a > res):
        res = a
print(res)