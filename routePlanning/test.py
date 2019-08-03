def foo(a):
    a.append("hi")

b = [1,2,3,4]
b = b + b
print(b)
for i in range(0, 5):
    print(i)
    i = i -1