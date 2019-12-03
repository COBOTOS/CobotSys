#include <stdio.h>

#include <string.h>

#include <curl/curl.h>
#include <iostream>
#include <boost/filesystem/path.hpp>
#include <logger/Logger.h>

//"http://localhost:8080/files/upload"
bool uploadFile(std::string url, std::string file, std::string license) {
    boost::filesystem::path p(file);
    CURL *curl;
    CURLcode res;

    struct curl_httppost *formpost = NULL;
    struct curl_httppost *lastptr = NULL;
    struct curl_slist *headerlist = NULL;
    license = "cobot_auth: " + license;
    headerlist = curl_slist_append(headerlist, license.data());
    const char buf[] = "Expect:";

    curl_global_init(CURL_GLOBAL_ALL);

    /* Fill in the file upload field */
    curl_formadd(&formpost,
                 &lastptr,
                 CURLFORM_COPYNAME, "file",
                 CURLFORM_FILE, file.data(),
                 CURLFORM_END);

    /* Fill in the filename field */
    curl_formadd(&formpost,
                 &lastptr,
                 CURLFORM_COPYNAME, "filename",
                 CURLFORM_COPYCONTENTS, p.filename().string().data(),
                 CURLFORM_END);

    /* Fill in the submit field too, even if this is rarely needed */
    curl_formadd(&formpost,
                 &lastptr,
                 CURLFORM_COPYNAME, "submit",
                 CURLFORM_COPYCONTENTS, "Submit",
                 CURLFORM_END);

    curl = curl_easy_init();
    /* initalize custom header list (stating that Expect: 100-continue is not
       wanted */
    headerlist = curl_slist_append(headerlist, buf);
    bool ret = false;
    if (curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.data());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
        curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);

        /* Perform the request, res will get the return code */
        res = curl_easy_perform(curl);
        /* Check for errors */
        if (res != CURLE_OK) {
            fprintf(stderr, "curl_easy_perform() failed: %s\n",
                    curl_easy_strerror(res));
        } else {
            long http_code = 0;
            curl_easy_getinfo (curl, CURLINFO_RESPONSE_CODE, &http_code);
            if (http_code == 200 && res != CURLE_ABORTED_BY_CALLBACK) {
                ret = true;
            } else {
                ret = false;
            }
        }

        /* always cleanup */
        curl_easy_cleanup(curl);

        /* then cleanup the formpost chain */
        curl_formfree(formpost);
        /* free slist */
        curl_slist_free_all(headerlist);
    }
    return ret;
}

int main(int argc, char *argv[]) {
    if (uploadFile("http://localhost:8080/files/upload",
                   "/media/cobot/cobotos/cobotos/Sparrow/Sparrow.Qt.Test/cmake-build-debug/uploadFile",
                   "eyJzaWciOiJ7XCJtYWNcIjpcIjMwMzAzQTMwMzAzQTMwMzAzQTMwMzAzQTMwMzAzQTMwMzBcIixcImNwdUlkXCI6XCIyOTU0OFwiLFwidXNlck5hbWVcIjpcImNvYm90LXVidW50dTE2XCIsXCJlbWFpbFwiOlwieHV6aGVuaGFpQGNvYm90c3lzLmNvbVwiLFwicGhvbmVcIjpudWxsLFwiZGVhZFRpbWVcIjoxNTcxOTAzMjA3NzAxfSIsImxpYyI6ImxBTDNtbFVNKzBzUnRlSng2Q3V2OFdQL0xLcFg2SE1BR2V3SnZFTDN2Y0FrdFh4OS9wTERhdlRHdHNUM3JTOCtlaWJUZkZ2NDVGcXlXa05GcEduUUlXWFIzR1pxMFFJU1JMRGo0ajNkOXRRRlBheDd0WWNPendHNDJseHJWOUVIOTY2by8vVEI1VHd2TFR4eXdxQXgxemg5U3lvR2YvUkFYL0ZsR0ZuZSt0bnpNdXIzbU5JbEQvanMwR0p0VEQ3NW1CdUxDTjBCWWRoMEVBeEVaN3EvVlZreFpLWElaR2w2R1YvMWRHVzFqVTdQOS9ZV1BBVWJHN2pXc0FKN0lKdUVLYkFwTUZqa24xRFAzSCtGUVNqMWovRDN1UUxGRFBGeFpwc0VvWUxaVzFyUkZYNFBpc25Fam11WTBIajI3RDdaWGZwclBwWGw4UnFSTk9KbmQ5eDdZZz09In0=")) {
        LOG_INFO << "upload sucess";
    }
}