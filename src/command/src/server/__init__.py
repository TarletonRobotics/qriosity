import SimpleHTTPServer
import SocketServer
import argparse

def main():
	PORT = 5000
	Handler = SimpleHTTPServer.SimpleHTTPRequestHandler

	httpd = SocketServer.TCPServer(("", PORT), Handler)

	print "serving at port", PORT
	httpd.serve_forever()

if __name__ == '__main__':
	main()