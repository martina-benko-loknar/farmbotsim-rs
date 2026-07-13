def summarize(df, group_cols, value_cols, count_col="seed"):
    """
    Aggregate a sweep results dataframe by `group_cols`, reporting mean and
    std (population of runs, i.e. across seeds) for each column in
    `value_cols`, plus the number of runs `n` per group.

    Example: summarize(df, ["soc_threshold"], ["energy_wh", "distance_m"])
    """

    agg = {}
    for col in value_cols:
        agg[f"{col}_mean"] = (col, "mean")
        agg[f"{col}_std"] = (col, "std")

    agg["n"] = (count_col, "count")

    return (
        df
        .groupby(group_cols)
        .agg(**agg)
        .reset_index()
    )
