import pandas
import matplotlib.pyplot as plt

def get_dataframe_from_xm(filename):
    df = pandas.read_xml(filename)
    return df

if __name__ == '__main__':
    df = get_dataframe_from_xm("sum.xml")
    print(df.iloc[100])
    df.plot(x="time",y="meanTravelTime")
    plt.show()