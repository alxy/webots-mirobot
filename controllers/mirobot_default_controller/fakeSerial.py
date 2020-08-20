import re

regex = r"M20 G91 G1( X(?P<x>[\d\.-]+))?( Y(?P<y>[\d\.-]+))?( Z(?P<z>[\d\.-]+))?( A(?P<a>[\d\.-]+))?( B(?P<b>[\d\.-]+))?( C(?P<c>[\d\.-]+))?( F(?P<f>[\d\.-]+))?"

test_str = "M20 G91 G1 Z-80 F2000"

matches = re.match(regex, test_str)
print(matches.groupdict())
params = matches.groupdict()

test = float(params.get('x') or 0)

print(test)
