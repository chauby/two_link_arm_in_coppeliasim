function err = calculateVWerr(Cref, Cnow)
    perr = Cref.p - Cnow.p;
    Rerr = Cnow.R^(-1) * Cref.R;
    werr = Cnow.R*rot2omega(Rerr);
    err = [perr; werr];
end