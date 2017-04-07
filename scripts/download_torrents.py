import urllib.request as url
from html.parser import HTMLParser
import re
import sys
import os

root = "http://www.rawseeds.org"
page = root + "/rs/capture_sessions/view/11"
document = "index.html"
download_folder = "/tmp"
document_path = download_folder + "/" + document

def reporthook(blocknum, blocksize, totalsize):
    readsofar = blocknum * blocksize
    if totalsize > 0:
        percent = readsofar * 1e2 / totalsize
        s = "\r%5.1f%% %*d / %d" % (
            percent, len(str(totalsize)), readsofar, totalsize)
        sys.stderr.write(s)
        if readsofar >= totalsize: # near the end
            sys.stderr.write("\n")
    else: # total size is unknown
        sys.stderr.write("read %d\n" % (readsofar,))


class RawseedsParser(HTMLParser):
    pattern = re.compile(".*\.torrent")
    statistic_only = False
    objects_found = 0
    objects_size = 0

    def set_statistic_only(self, b):
        self.statistic_only = b

    def handle_starttag(self, tag, attrs):
        for a in attrs:
            if a[0] == 'href':
                if self.pattern.match(a[1]):
                    self.download(a[1])

    def handle_endtag(self, tag):
        return

    def handle_data(self, data):
        return

    def get_id_and_file(self, link):
        tokens = link.split('/')
        return (tokens[-2], tokens[-1])

    def download(self, link):
        file_link = root + "/" + link
        print("Opening : " + file_link)
        meta = url.urlopen(file_link).info()
        self.objects_found += 1

        if not self.statistic_only:
            # get object id
            print(link)
            id, file = self.get_id_and_file(link)
            # check if folder is there
            object_path = download_folder + "/"
            if not os.path.exists(object_path):
                os.makedirs(object_path)
            # download it
            out_file = object_path + "/" + file
            url.urlretrieve(file_link, out_file, reporthook)


# get the file
url.urlretrieve(page + "/" + document, document_path, reporthook)
html = open(document_path, 'r').read()
parser = RawseedsParser()
parser.set_statistic_only(False)
parser.feed(html)