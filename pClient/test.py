def main():
    d = {
        5 : "batata"
    } 
    asd = ""
    for c in d.get(5):
     if c == "b":
        asd += "a"
     else:
         asd += c
    d[5] = asd

    print(d.get(5))

    print(d.get(5))
if __name__ == "__main__":
    main()