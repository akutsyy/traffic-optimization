import pandas

def get_dataframe_from_xm(filename):
    df = pandas.read_xml(filename)
    print(df.iloc[100])

if __name__ == '__main__':
    get_dataframe_from_xm("sum.xml")