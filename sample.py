import argparse
from dataclasses import dataclass
from datetime import datetime
import logging
from lxml import etree
import os
import sqlite3

@dataclass
class DataPoint:
    timestamp: int   = None
    temp:      float = None
    humidity:  float = None
    co2:       int   = None

    @classmethod
    def from_xml(cls, line):
        xml = line.strip()
        if not line.startswith("<xml"):
            return None

        try:
            dp = cls()
            root = etree.fromstring(xml)
            
            try:
                dp.timestamp = root.attrib.get('ts')
                int(dp.timestamp)
            except:
                logging.warning("Invalid 'ts' attribute: %s" % (dp.timestamp))
                return None

            elem = root.xpath('temp_f')
            if elem is not None:
                dp.temp = elem[0].text
            elem = root.xpath('humidity')
            if elem is not None:
                dp.humidity = elem[0].text
            elem = root.xpath('co2')
            if elem is not None:
                dp.co2 = elem[0].text

            return dp
        except:
            logging.exception("Error parsing '%s'" % (xml))
        
        return None


def import_xml_file(db_filename, xml_filename) -> int:
    if not os.path.exists(db_filename):
        with open(db_filename, 'w') as f:
            pass
            
        with sqlite3.connect(db_filename) as db:
            db.execute("""CREATE TABLE dayclock
                          (
                             timestamp   INT     PRIMARY KEY,
                             temp        REAL    NULL,
                             humidity    REAL    NULL,
                             co2         INT     NULL
                          )""")

    i = 0
    with sqlite3.connect(db_filename) as db:
        with open(xml_filename) as f:
            for line in f.readlines():
                dp = DataPoint.from_xml(line)
                if dp:
                    db.execute("REPLACE INTO dayclock (timestamp, temp, humidity, co2) VALUES (?, ?, ?, ?)", (dp.timestamp, dp.temp, dp.humidity, dp.co2))
                    i += 1
    return i


def output_csv(db_filename, output_csv_filename) -> (int,int):
    i = 0
    skipped = 0
    with sqlite3.connect(db_filename) as db:
        with open(output_csv_filename, 'w') as f:
            f.write('Date, Timestamp, Temperature, Humidity, CO2\n');
            for row in db.execute('SELECT timestamp, temp, humidity, co2 FROM dayclock ORDER BY timestamp'):
                if row[3] < 350 or row[3] > 2000:
                    skipped += 1
                    continue
                f.write('"%s",' % (datetime.fromtimestamp(row[0])))
                f.write("%s\n" % (','.join('"%s"' % (str(x)) for x in row)))
                i += 1
    return i, skipped


if __name__ == "__main__":
    DB_FILENAME         = 'day.db'
    XML_FILENAME        = 'data.xml'
    OUTPUT_CSV_FILENAME = 'dayclock.csv'

    db_filename = DB_FILENAME
    input_xml_filename = None
    output_csv_filename = OUTPUT_CSV_FILENAME
    
    parser = argparse.ArgumentParser(description='Read input Dayclock XML file and update the database')
    parser.add_argument('input_xml_file', type=str, help='Input XML filename')
    parser.add_argument('--db_filename', type=str, metavar='filename', required=False, help='SQLite database filename')
    parser.add_argument('--output_csv_filename', type=str, metavar='filename', required=False, help='Output CSV filename')
    args = parser.parse_args()
    
    input_xml_filename = args.input_xml_file
    if not input_xml_filename or not os.path.exists(input_xml_filename):
        print("Missing or invalid input_xml_filename")
        raise FileNotFoundError(input_xml_filename)
    if args.db_filename:
        db_filename = args.db_filename
    if args.output_csv_filename:
        output_csv_filename = args.output_csv_filename
        
    print("Staring DB import from %s" % (input_xml_filename))
    count = import_xml_file(db_filename, input_xml_filename)
    print("%s records imported into %s" % (count, db_filename))
    
    print("Starting CSV export from %s" % (db_filename))
    count, skipped = output_csv(db_filename, output_csv_filename)
    print("%s records (%s skipped) written to %s" % (count, skipped, output_csv_filename))
