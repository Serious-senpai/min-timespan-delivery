SELECT problem, MIN(cost), AVG(elapsed)
FROM summary
GROUP BY problem;
