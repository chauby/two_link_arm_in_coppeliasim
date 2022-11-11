function idx = findRoute(to)
    global uLINK

    i = uLINK(to).mother;
    if i == 1
        idx = [to];
    else
        idx = [findRoute(i)  to];
    end
end